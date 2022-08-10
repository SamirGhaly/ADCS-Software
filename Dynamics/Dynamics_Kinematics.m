function stateout=Dynamics_Kinematics(Orbit,IC,Tspan,timestep)
global CTRL Control_Method B_ECI m_vec B_Body Bx_ECI By_ECI Bz_ECI Bx_Body By_Body Bz_Body B_ECEF Bx_ECEF By_ECEF Bz_ECEF B_Orbit Bx_Orbit By_Orbit Bz_Orbit
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

% Angles, Quaternion and Angular Velocity
Angles0 = [IC.phi0;IC.theta0;IC.psi0]*pi/180;
q0123_0 = EulerAngles2Quaternions(Angles0);
p0=IC.p0;
q0=IC.q0;
r0=IC.r0;
state = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];


%% Simulation 
% Parameters
stateout = zeros(length(state),length(Tspan));
mu_x=0*stateout(1,:);
mu_y=mu_x;
mu_z=mu_x;
Bx_ECI = 0*stateout(1,:);
By_ECI = Bx_ECI;
Bz_ECI = Bx_ECI;

%%%%Call the Derivatives Routine to initialize vars
k1 = Satellite(Tspan(1),state,1);
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
    %%%Save the current state
    stateout(:,idx) = state;
   %fprintf("[%.2f,%.2f,%.2f]  [%.2f,%.2f,%.2f]\n",Quaternions2EulerAngles(state(7:10))*180/pi,state(11:13)*180/pi)
   % fprintf("[%.2f]\n",sqrt(sum(state(7:10).^2)))
    %%%%Then we make our 4 function calls for the RK4
  %  fprintf("%.5f %.5f\n",Tspan(idx),norm(state(11:13))*180/pi) 
  %% Switch Controller
%     if Tspan(idx)==CTRL.SWITCH
%         Control_Method=1;
%     end
    %% RK4
    k1 = Satellite(Tspan(idx),state,1);
    k2 = Satellite(Tspan(idx)+timestep/2,state+k1*timestep/2,2);
    k3 = Satellite(Tspan(idx)+timestep/2,state+k2*timestep/2,3);
    k4 = Satellite(Tspan(idx)+timestep,  state+k3*timestep  ,4);

    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    state = state + k*timestep;
    state(7:10) = state(7:10) *sign(state(7)) / norm(state(7:10)) ;
end
disp('Simulation Complete')

%%%Plot Magnetic Field [ECI]
figure
hold on
plot(Tspan,Bx_ECI,'r-','LineWidth',1.5);
plot(Tspan,By_ECI,'g-','LineWidth',1.5);
plot(Tspan,Bz_ECI,'b-','LineWidth',1.5);
xlabel('Time (sec)','FontSize',12)
ylabel('Magnetic Field Strength (nT)','FontSize',12)
legend('$B_x$','$B_y$','$B_z$','Interpreter','latex','FontSize',10)
grid on
hold off

%%%Plot Magnetic Field [Body]
figure
hold on
plot(Tspan,Bx_Body,'r-','LineWidth',1.5);
plot(Tspan,By_Body,'g-','LineWidth',1.5);
plot(Tspan,Bz_Body,'b-','LineWidth',1.5);
xlabel('Time (sec)','FontSize',12)
ylabel('Magnetic Field Strength in Body frame (nT)','FontSize',12)
legend('$B_x$','$B_y$','$B_z$','Interpreter','latex','FontSize',12)
grid on
hold off

%%%Control Action
figure
title('Magnetic Dipole Moment')

subplot(3,1,1)
hold on
grid on
plot(Tspan,mu_x,'r-','LineWidth',1);
xlabel('Time (sec)','FontSize',12)
ylabel('N.m','FontSize',12)
legend('$\mu_X$','Interpreter','latex','FontSize',12)
hold off

subplot(3,1,2)
hold on
grid on
plot(Tspan,mu_y,'g-','LineWidth',1);
xlabel('Time (sec)','FontSize',12)
ylabel('N.m','FontSize',12)
legend('$\mu_Y$','Interpreter','latex','FontSize',12)
hold off

subplot(3,1,3)
hold on
grid on
plot(Tspan,mu_z,'b-','LineWidth',1);
xlabel('Time (sec)','FontSize',12)
ylabel('N.m','FontSize',12)
legend('$\mu_Z$','Interpreter','latex','FontSize',12)
hold off

%%%Desired Torque
% figure
% title('Actual Torque vs Desired Torque')
% hold on
% plot(Tspan,Tx,'r-','LineWidth',0.5);
% plot(Tspan,Ty,'g-','LineWidth',0.5);
% plot(Tspan,Tz,'b-','LineWidth',0.5);
% % plot(Tspan,Tx_des,'r--','LineWidth',0.5);
% % plot(Tspan,Ty_des,'g--','LineWidth',0.5);
% % plot(Tspan,Tz_des,'b--','LineWidth',0.5);
% xlabel('Time (sec)','FontSize',15)
% ylabel('Torque (N.m)','FontSize',15)
% legend('$T_x$','$T_y$','$T_z$','Interpreter','latex','FontSize',10)
% grid on
% hold off
end




