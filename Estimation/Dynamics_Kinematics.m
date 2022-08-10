function [stateout ,stateout_corr]=Dynamics_Kinematics(Orbit,IC,Tspan,timestep)
global Control_Method B_ECI m_vec LMN_mgtorquers Pulses T_des B_Body Bx_ECI By_ECI Bz_ECI Bx_Body By_Body Bz_Body B_ECEF Bx_ECEF By_ECEF Bz_ECEF B_Orbit Bx_Orbit By_Orbit Bz_Orbit Inertia w_O q_B_I q_O_I dt F state_corr
%%%%Simulation of a Low Earth Satellite
disp('Simulation Started')
%% Intitial States
%Position & Velocity
[R_ECI, V_ECI] = sv_from_coe(Orbit);
x0=R_ECI(1);
y0=R_ECI(2);
z0=R_ECI(3);
xdot0=V_ECI(1);
ydot0=V_ECI(2);
zdot0=V_ECI(3);
%..............
%..............
% Angles, Quaternion and Angular Velocity
Angles0 = [IC.phi0;IC.theta0;IC.psi0]*pi/180;
q0123_0 = EulerAngles2Quaternions(Angles0);
% q1=q0123_0(1);q2=q0123_0(2);q3=q0123_0(3);q4=q0123_0(4);
p0=IC.p0;
q0=IC.q0;
r0=IC.r0;
state = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];
state_corr=state; %corrected state
%..........................
stateout=zeros(length(state),length(Tspan));
stateout_corr=zeros(length(state_corr),length(Tspan));
stateout_actual=zeros(length(state),length(Tspan));
Magnetic_Field=zeros(3,length(Tspan));
Euler_Corrected=zeros(3,length(Tspan));
Euler_Actual=zeros(3,length(Tspan));
% P=[(100e-8)*eye(4,4)    zeros(4,3);
%          zeros(3,4)         1e-10*eye(3)];
P=[(100e-1)*eye(4,4)    zeros(4,3);
         zeros(3,4)         100*eye(3)];
%Process Noise
% system_noise=5e-3;
% i_xx=0.00216666666666667;
% i_yy=i_xx;
% i_zz=i_xx;
% Q=diag([0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0.0001]);
%Measurment Noise R
% quat_sigma=[1; 1;1;1 ]*500e-9;omega_sigma=[1;1;1]*8.727e-4;
quat_sigma=[1; 1;1;1 ]*5e-2;omega_sigma=[1;1;1]*8.727e-4;
R(1,1)=quat_sigma(1);R(2,2)=quat_sigma(2);R(3,3)=quat_sigma(3);R(4,4)=quat_sigma(4);
R(5,5)=omega_sigma(1);R(6,6)=omega_sigma(2);R(7,7)=omega_sigma(3);

%% Simulation 
% Parameters
stateout = zeros(length(state),length(Tspan));
mu_x=0*stateout(1,:);
mu_y=mu_x;
mu_z=mu_x;
Bx_ECI = 0*stateout(1,:);
By_ECI = Bx_ECI;
Bz_ECI = Bx_ECI;
Euler_angles = [IC.phi0,IC.theta0,IC.psi0]*pi/180;
Angles_corr=[10,10,10]*pi/180;
% q1234 = eul2q(Euler_angles,'xyz','wxyz');
% pqr=[IC.p0;IC.q0;IC.r0];
%%%%Call the Derivatives Routine to initialize vars
k1 = Satellite(Tspan(1),state,1);
A_magnetometer=eye(3);
sigma_quat=5e-2; 
n_magnetometer=normrnd(0,sigma_quat,[3,1]);
A_gyro=eye(3);
sigma_omega=8.727e-4; 
% sigma_omega=5e-8; 
system_noise=1e-5;
  n_omega=normrnd(0,sigma_omega,[3,1]);
  S=system_noise^2*eye(3,3);
  Q1=[zeros(4,4) zeros(4,3);zeros(3,4) S]    ;
 for idx = 1:length(Tspan)
    %%%Save the magnetic field
    Bx_ECI(idx) = B_ECI(1);
    By_ECI(idx) = B_ECI(2);
    Bz_ECI(idx) = B_ECI(3);
    Bx_Body(idx) = B_Body(1);
    By_Body(idx) = B_Body(2);
    Bz_Body(idx) = B_Body(3);
    Bx_ECEF(idx) = B_ECEF(1);
    By_ECEF(idx) = B_ECEF(2);
    Bz_ECEF(idx) = B_ECEF(3);
    Bx_Orbit(idx) = B_Orbit(1);
    By_Orbit(idx) = B_Orbit(2);
    Bz_Orbit(idx) = B_Orbit(3);
    mu_x(idx)=m_vec(1);
    mu_y(idx)=m_vec(2);
    mu_z(idx)=m_vec(3);
    Tx(idx)=LMN_mgtorquers(1);
    Ty(idx)=LMN_mgtorquers(2);
    Tz(idx)=LMN_mgtorquers(3);
%     Tx_des(idx)=T_des(1);
%     Ty_des(idx)=T_des(2);
%     Tz_des(idx)=T_des(3);
    %%%Save the current state
    stateout(:,idx) = state;
    stateout_corr(:,idx) = state_corr;
    stateout_actual(:,idx) = state;
    Euler_Actual(:,idx)=Euler_angles';
    Euler_Corrected(:,idx)=Angles_corr';
   %fprintf("[%.2f,%.2f,%.2f]  [%.2f,%.2f,%.2f]\n",Quaternions2EulerAngles(state(7:10))*180/pi,state(11:13)*180/pi)
   % fprintf("[%.2f]\n",sqrt(sum(state(7:10).^2)))
    %%%%Then we make our 4 function calls for the RK4
  %  fprintf("%.5f %.5f\n",Tspan(idx),norm(state(11:13))*180/pi) 
  %% Switch Controller
  %   if Tspan(idx)==7000
  %       Control_Method=1;
  %   end
    %% RK4
%     k1 = Satellite(Tspan(idx),state,1);
%     k2 = Satellite(Tspan(idx)+timestep/2,state+k1*timestep/2,2);
%     k3 = Satellite(Tspan(idx)+timestep/2,state+k2*timestep/2,3);
%     k4 = Satellite(Tspan(idx)+timestep,  state+k3*timestep  ,4);
%     k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
%     state = state + k*timestep;
    k1 = Satellite(Tspan(idx),state,1);
    k2 = Satellite(Tspan(idx)+timestep/2,state+k1*timestep/2,2);
    k3 = Satellite(Tspan(idx)+timestep/2,state+k2*timestep/2,3);
    k4 = Satellite(Tspan(idx)+timestep,  state+k3*timestep  ,4);
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    state = state + k*timestep;
     state(7:10) = state(7:10) *sign(state(7)) / norm(state(7:10)) ;
    %..................................................................
    q1=state_corr(7);
    q2=state_corr(8);
    q3=state_corr(9);
    q4=state_corr(10);
    p=state_corr(11);
    q=state_corr(12);
    r=state_corr(13);
    LMN_mgtorquers = [0;0;0]*10e-5;
    %State Transition Matrix using Prvious Corrected States:
    [state_transition,F]=state_trans(Inertia,p,q,r,w_O,q1,q2,q3,q4,timestep);
    F12=F(1:4,5:7);
     F22=F(5:7,5:7);
     MM=F12*S;
    Q2=[zeros(4,4) MM;(F12*S)'  F22'*S+F22*S];
     Q3=[F12*S*F12' F12*S*F22';F22*S*F12' F22*S*F22'];
     Q=Q1*dt+Q2*(dt)^2/2+Q3*(dt)^3/3;
    %Predicted Covarience
    P_pred=state_transition*P*state_transition'+Q;
    %Calculate Predicted States using RK4 (input is corrected state from
    %last iteration)
%     k1 = Satellite(Tspan(idx),state_corr,1);
%     k2 = Satellite(Tspan(idx)+timestep/2,state_corr+k1*timestep/2,2);
%     k3 = Satellite(Tspan(idx)+timestep/2,state_corr+k2*timestep/2,3);
%     k4 = Satellite(Tspan(idx)+timestep,state_corr+k3*timestep,4);
%     k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    k1 = Satellite(Tspan(idx),state_corr,1);
    k2 = Satellite(Tspan(idx)+timestep/2,state_corr+k1*timestep/2,2);
    k3 = Satellite(Tspan(idx)+timestep/2,state_corr+k2*timestep/2,3);
    k4 = Satellite(Tspan(idx)+timestep,state_corr+k3*timestep,4);
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);

    state_corr = state_corr + k*timestep;
   state_corr(7:10) = state_corr(7:10) *sign(state_corr(7)) / norm(state_corr(7:10)) ;
    %%CORRECTION CYCLE
    %get new observation
    %Sensor Model
    %GYRO
    w_d=A_gyro*state(11:13)+n_omega;
    w_obs=inv(A_gyro)*w_d;
    %Magnetometer
%     Euler_angles=wrapTo2Pi(q2eul(state(7:10),'wxyz','xyz'));
%     state(7:10)=eul2q(Euler_angles,'xyz','wxyz');
%     q_B_O=state(7:10);
%     [B_ECI,~,~,~]=igrf_Inertia(R_ECI);
%     B_Orbit_nT=newfrm(B_ECI,q_O_I);
%     B_Orbit=B_Orbit_nT*1e-9;
%     B_Body_nT=newfrm(B_ECI,q_B_I);
%     B_Body=B_Body_nT*1e-9;

        %Extract Quaternion from Magnetic Field
        x0 = Euler_angles;
        fun = @(x)getDCM(x,B_Orbit,B_Body+n_magnetometer);
        Angles_corr=wrapTo2Pi(fsolve(fun,x0));
        q_obs=eul2q(Angles_corr,'xyz','wxyz');
%   q_obs=A_magnetometer*state(7:10)+n_magnetometer;
        Magnetic_Field(:,idx)=B_Body;
        z_obs=[q_obs; w_obs];
    %Observation Equation H 
        H=eye(7);
    %Kalman Gain
       SS=H*P_pred*H'+R;
       K=P_pred*(H')*inv(SS);
    %get corrected state
    z_corr=state(7:13)+K*(z_obs-state(7:13));
    state_corr=[state(1:6);z_corr];
    %update Error Covariance
    P=(eye(7)-K*H)*P_pred;
end
disp('Simulation Complete')
p_corr=stateout_corr(11,:);
q_corr=stateout_corr(12,:);
r_corr=stateout_corr(13,:);
p_actual=stateout_actual(11,:);
 q_actual=stateout_actual(12,:);
 r_actual=stateout_actual(13,:);


figure(1)

subplot(3,1,1,'align')
plot(Tspan,p_corr,'r',Tspan,p_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),p_corr(1),'r*',Tspan(1),p_actual(1),'b*')
legend('p corrected','p actual','Initial Estimate','Initial Actual')

subplot(3,1,2,'align')
plot(Tspan,q_corr,'r',Tspan,q_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),q_corr(1),'r*',Tspan(1),q_actual(1),'b*')
legend('q corrected','q actual','Initial Estimate','Initial Actual')

subplot(3,1,3,'align')
plot(Tspan,r_corr,'r',Tspan,r_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),r_corr(1),'r*',Tspan(1),r_actual(1),'b*')
legend('r corrected','r actual','Initial Estimate','Initial Actual')


q1_corr=stateout_corr(7,:);
q2_corr=stateout_corr(8,:);
q3_corr=stateout_corr(9,:);
q4_corr=stateout_corr(10,:);
q1_actual=stateout_actual(7,:);
q2_actual=stateout_actual(8,:);
q3_actual=stateout_actual(9,:);
q4_actual=stateout_actual(10,:);

% for idx = 1:length(Tspan)
%     if q1_corr(idx)<0
%         q1_corr(idx)=-1*q1_corr(idx);
%     end    
%     if q1_actual(idx)<0
%         q1_actual(idx)=-1*q1_actual(idx);
%     end    
% end

figure(2)
subplot(4,1,1,'align')
plot(Tspan,q1_corr,'r',Tspan,q1_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),q1_corr(1),'r*',Tspan(1),q1_actual(1),'b*')
legend('q1 corrected','q1 actual','Initial Estimate','Initial Actual')

subplot(4,1,2,'align')
plot(Tspan,q2_corr,'r',Tspan,q2_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),q2_corr(1),'r*',Tspan(1),q2_actual(1),'b*')
legend('q2 corrected','q2 actual','Initial Estimate','Initial Actual')


subplot(4,1,3,'align')
plot(Tspan,q3_corr,'r',Tspan,q3_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),q3_corr(1),'r*',Tspan(1),q3_actual(1),'b*')
legend('q3 corrected','q3 actual','Initial Estimate','Initial Actual')


subplot(4,1,4,'align')
plot(Tspan,q4_corr,'r',Tspan,q4_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),q4_corr(1),'r*',Tspan(1),q4_actual(1),'b*')
legend('q4 corrected','q4 actual','Initial Estimate','Initial Actual')


error_q1=q1_corr-q1_actual;
error_q2=q2_corr-q2_actual;
error_q3=q3_corr-q3_actual;
error_q4=q4_corr-q4_actual;
figure(3)
subplot(4,1,1,'align')
plot(Tspan,error_q1,'LineWidth',1.5)
legend('q1 error')
subplot(4,1,2,'align')
plot(Tspan,error_q2,'LineWidth',1.5)
legend('q2 error')
subplot(4,1,3,'align')
plot(Tspan,error_q3,'LineWidth',1.5)
legend('q3 error')
subplot(4,1,4,'align')
plot(Tspan,error_q4,'LineWidth',1.5)
legend('q4 error')

error_p=p_corr-p_actual;
error_q=q_corr-q_actual;
error_r=r_corr-r_actual;


figure(4)
subplot(3,1,1,'align')
plot(Tspan,error_p,'LineWidth',1.5)
legend('p error')
subplot(3,1,2,'align')
plot(Tspan,error_q,'LineWidth',1.5)
legend('q error')
subplot(3,1,3,'align')
plot(Tspan,error_r,'LineWidth',1.5)
legend('r error')

figure(5)
Bx=Magnetic_Field(1,:);
By=Magnetic_Field(2,:);
Bz=Magnetic_Field(3,:);
subplot(3,1,1,'align')
plot(Tspan,Bx,'LineWidth',1.5)
legend('B_x')
subplot(3,1,2,'align')
plot(Tspan,By,'LineWidth',1.5)
legend('B_y')
subplot(3,1,3,'align')
plot(Tspan,Bz,'LineWidth',1.5)
legend('B_z')

figure(6)
phi_corr=Euler_Corrected(1,:);
theta_corr=Euler_Corrected(2,:);
epsi_corr=Euler_Corrected(3,:);
phi_actual=Euler_Actual(1,:);
theta_actual=Euler_Actual(2,:);
epsi_actual=Euler_Actual(3,:);
subplot(3,1,1,'align')
plot(Tspan,phi_corr,'r',Tspan,phi_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),phi_corr(1),'r*',Tspan(1),phi_actual(1),'b*')
legend('phi corrected','phi actual','Initial Estimate','Initial Actual')

subplot(3,1,2,'align')
plot(Tspan,theta_corr,'r',Tspan,theta_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),theta_corr(1),'r*',Tspan(1),theta_actual(1),'b*')
legend('theta corrected','theta actual','Initial Estimate','Initial Actual')

subplot(3,1,3,'align')
plot(Tspan,epsi_corr,'r',Tspan,epsi_actual,'--b','LineWidth',1.5)
hold on
plot(Tspan(1),epsi_corr(1),'r*',Tspan(1),epsi_actual(1),'b*')
legend('epsi corrected','epsi actual','Initial Estimate','Initial Actual')

figure(7)
error_phi=phi_corr-phi_actual;
error_theta=theta_corr-theta_actual;
error_epsi=epsi_corr-epsi_actual;
subplot(3,1,1,'align')
plot(Tspan,error_phi,'LineWidth',1.5)
legend('phi error')
subplot(3,1,2,'align')
plot(Tspan,error_theta,'LineWidth',1.5)
legend('theta error')
subplot(3,1,3,'align')
plot(Tspan,error_epsi,'LineWidth',1.5)
legend('epsi error')
% %%%Plot Magnetic Field [ECI]
% figure
% hold on
% plot(Tspan,Bx_ECI,'r-','LineWidth',1.5);
% plot(Tspan,By_ECI,'g-','LineWidth',1.5);
% plot(Tspan,Bz_ECI,'b-','LineWidth',1.5);
% xlabel('Time (sec)','FontSize',15)
% ylabel('Magnetic Field Strength (nT)','FontSize',15)
% legend('$B_x$','$B_y$','$B_z$','Interpreter','latex','FontSize',10)
% grid on
% hold off
% 
% %%%Plot Magnetic Field [Body]
% figure
% hold on
% plot(Tspan,Bx_Body,'r-','LineWidth',1.5);
% plot(Tspan,By_Body,'g-','LineWidth',1.5);
% plot(Tspan,Bz_Body,'b-','LineWidth',1.5);
% xlabel('Time (sec)','FontSize',15)
% ylabel('Magnetic Field Strength in Body frame (nT)','FontSize',15)
% legend('$B_x$','$B_y$','$B_z$','Interpreter','latex','FontSize',10)
% grid on
% hold off
% 
% %%%Control Action
figure
title('Magnetic Dipole Moment')

subplot(3,1,1)
hold on
grid on
plot(Tspan,mu_x,'r-','LineWidth',0.5);
xlabel('Time (sec)','FontSize',15)
ylabel('N.m','FontSize',15)
legend('$M_x$','Interpreter','latex','FontSize',10)
hold off

subplot(3,1,2)
hold on
grid on
plot(Tspan,mu_y,'g-','LineWidth',0.5);
xlabel('Time (sec)','FontSize',15)
ylabel('N.m','FontSize',15)
legend('$M_y$','Interpreter','latex','FontSize',10)
hold off

subplot(3,1,3)
hold on
grid on
plot(Tspan,mu_z,'b-','LineWidth',0.5);
xlabel('Time (sec)','FontSize',15)
ylabel('N.m','FontSize',15)
legend('$M_z$','Interpreter','latex','FontSize',10)
hold off

%%%Desired Torque
% % figure
% % title('Actual Torque vs Desired Torque')
% % hold on
% % plot(Tspan,Tx,'r-','LineWidth',0.5);
% % plot(Tspan,Ty,'g-','LineWidth',0.5);
% % plot(Tspan,Tz,'b-','LineWidth',0.5);
% % % plot(Tspan,Tx_des,'r--','LineWidth',0.5);
% % % plot(Tspan,Ty_des,'g--','LineWidth',0.5);
% % % plot(Tspan,Tz_des,'b--','LineWidth',0.5);
% % xlabel('Time (sec)','FontSize',15)
% % ylabel('Torque (N.m)','FontSize',15)
% % legend('$T_x$','$T_y$','$T_z$','Interpreter','latex','FontSize',10)
% % grid on
% % hold off
end




