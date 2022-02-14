function dstatedt = Satellite(t,state)
%%%stateinitial = [x0;y0;z0;xdot0;ydot0;zdot0];
global invI I m 
x = state(1);
y = state(2);
z = state(3);
%xdot = state(4);
%ydot = state(5);
%zdot = state(6);
q0123 = state(7:10);
p = state(11);
q = state(12);
r = state(13);
pqr = state(11:13);
%%%Translational Kinematics
vel = state(4:6);
%%%Rotational Kinematics
PQRMAT = [0 -p -q -r;p 0 r -q;q -r 0 p;r q -p 0];
q0123dot = 0.5*PQRMAT*q0123;
%%%Gravity Model
planet
r = state(1:3); %% r = [x;y;z]
rho = norm(r);
rhat = r/rho;
Fgrav = -(G*M*m/rho^2)*rhat;

%%%Translational Dynamics
F = Fgrav;   %Forces in Inertial Frame
accel = F/m;

%%%Magtorquer Model
LMN_magtorquers = [0;0;0];

%%%Rotational Dynamics
H = I*pqr;   
pqrdot = invI*(LMN_magtorquers - cross(pqr,H));

%%%Return derivatives vector
dstatedt = [vel;accel;q0123dot;pqrdot];

