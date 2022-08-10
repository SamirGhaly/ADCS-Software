function dstatedt = Satellite(t,state,step)
Planet.Mass = 5.972e24; %%Kg
Planet.Const = 6.6743e-20;  %%%some SI unit
Planet.mu = 3.986004418e5 ; %km^3.s^-2

global w_O Sat Orbit dt Control_Method q_desired M_max Parameters date Inertia B_ECI m_vec LMN_mgtorquers Pulses T_des B_Body B_Orbit 
x = state(1);
y = state(2);
z = state(3);
R_ECI = state(1:3);
%xdot = state(4);
%ydot = state(5);
%zdot = state(6);
V_ECI=state(4:6);
q_B_O = state(7:10);
p = state(11); 
q = state(12);
r = state(13);



pqr =[p;q;r];   %Omega_B_I  [rad/s]

%%%Translational Kinematics
vel = state(4:6);

%%%Rotational Kinematics
PQRMAT = [0 -p -q -r;p 0 r -q;q -r 0 p;r q -p 0];
%q0123dot = 0.5*PQRMAT*q_B_I;   q_B_I_dot
q_B_O_dot=0.5*(quatmul(q_B_O,[0;p;q;r])-quatmul([0;0;-w_O;0],q_B_O));


%Quaternions
q_O_I=I2O(R_ECI,V_ECI);
q_I_O=conjq(q_O_I);
q_B_I=quatmul(q_O_I,q_B_O);

delta_q=q_error(q_B_O,q_desired);

%% Gravity Model
% rho = norm(R_ECI);
% rhat = R_ECI/rho;
% Fgrav=-(Planet.mu*Sat.m/rho^2)*rhat;
% g=Fgrav/Planet.Mass;

g= gravity(x,y,z,date);
%% Magnetic Field
%B_ECI=igrf_Inertia(R_ECI);
[B_ECI,~,~,~]=igrf_Inertia(R_ECI);
B_Orbit=newfrm(B_ECI,q_O_I);
B_Body_nT = newfrm(B_ECI,q_B_I);

%%%Convert to Tesla
B_Body = B_Body_nT*1e-9;  

%% 
% SV_Body=SunV_BodyFrame(q_B_I);
% V_Orbit=newfrm(V_ECI,q_O_I);

%% Disturbances
%[F_D,M_D] =Optimized_Dis(R_ECI,Inertia,SV_Body,date,latitude,longitude,altitude*1e-3,vel,B_ECI);
F_D = [0;0;0] ;
M_D = [0;0;0] ;

 %% Translational Dynamics
accel_D = F_D/Planet.Mass;
accel=g+accel_D;

%% Control
if step == 1
[mu,T_des] = control(delta_q,pqr,Control_Method,q_desired,B_Body,q_B_O);
Pulses     = PWM(mu,M_max);
[LMN_mgtorquers,m_vec] = Magnetorquer_NOPWM(B_Body,mu,M_max);
end
%% Magtorquer Model

%[LMN_mgtorquers,m_vec] = Magnetorquer(B_Body,Pulses,mu,M_max);


%% Rotational Dynamics
H = Inertia*pqr;

pqrdot = (Inertia)^-1*(LMN_mgtorquers - cross(pqr,H)+M_D);      %I^-1 [ -Skew(w)(I*w)+Ndist+Nctrl]

%% Return derivatives vector
dstatedt = [vel;accel;q_B_O_dot;pqrdot];
end
