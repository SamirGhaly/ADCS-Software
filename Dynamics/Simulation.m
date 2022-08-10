
addpath('github_repo\planet3D\')

%%% Extract the state vector
Rx_ECI = stateout(1,:);
Ry_ECI = stateout(2,:);
Rz_ECI = stateout(3,:);
Vx_ECI = stateout(4,:);
Vy_ECI = stateout(5,:);
Vz_ECI = stateout(6,:);
q0123out = stateout(7:10,:);
Angles_out = Quaternions2EulerAngles(q0123out);  %rad
Phi=Angles_out(1,:)*180/pi;
Theta=Angles_out(2,:)*180/pi;
Psi=Angles_out(3,:)*180/pi;
pqr_out = stateout(11:13,:);
p=pqr_out(1,:)*180/pi;
q=pqr_out(2,:)*180/pi;
r=pqr_out(3,:)*180/pi;

%% Plot
%%Plot X,Y,Z as a function of time
fig0 = figure();
set(fig0,'color','white')
plot(Tspan,Rx_ECI,'r-','LineWidth',2)
hold on
grid on
plot(Tspan,Ry_ECI,'g-','LineWidth',2)
plot(Tspan,Rz_ECI,'b-','LineWidth',2)
xlabel('Time (sec)')
ylabel('Position (Km)')
legend('X','Y','Z')

%%%Plot Trajectory
fig1 = figure();
hold on
planet3D('Earth');
set(fig1,'color','white')
plot3(Rx_ECI,Ry_ECI,Rz_ECI,'b-','LineWidth',4)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold off
axis equal

%% Plot Euler Angles
figure
hold on
grid on
plot(Tspan,Phi,'r-','LineWidth',2)
plot(Tspan,Theta,'g-','LineWidth',2)
plot(Tspan,Psi,'b-','LineWidth',2)
legend('$\phi$','$\theta$','$\psi$','Interpreter','latex','FontSize',12)
xlabel('Time (sec)','FontSize',12)
ylabel('Angles (deg)','FontSize',12)

hold off
%% Plot Euler Angles Splitted
% figure
% subplot(3,1,1)
% hold on
% grid on
% xlabel('Time (sec)','Interpreter','latex','FontSize',12)
% ylabel('$\phi (deg)$','Interpreter','latex','FontSize',12)
% plot(Tspan,Phi,'r-','LineWidth',2)
% hold off
% 
% subplot(3,1,2)
% hold on
% grid on
% xlabel('Time (sec)','Interpreter','latex','FontSize',12)
% ylabel('$\theta (deg)$','Interpreter','latex','FontSize',12)
% plot(Tspan,Theta,'g-','LineWidth',2)
% hold off
% 
% subplot(3,1,3)
% hold on
% grid on
% xlabel('Time (sec)','Interpreter','latex','FontSize',12)
% ylabel('$\psi (deg)$','Interpreter','latex','FontSize',12)
% plot(Tspan,Psi,'b-','LineWidth',2)
% hold off


%% Plot Quaternions
figure

subplot(4,1,1)
hold on 
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('q0','FontSize',12)
plot(Tspan,q0123out(1,:),'k','LineWidth',1)
hold off

subplot(4,1,2)
hold on 
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('q1','FontSize',12)
plot(Tspan,q0123out(2,:),'r','LineWidth',1)
hold off

subplot(4,1,3)
hold on 
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('q2','FontSize',12)
plot(Tspan,q0123out(3,:),'g','LineWidth',1)
hold off

subplot(4,1,4)
hold on 
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('q3','FontSize',12)
plot(Tspan,q0123out(4,:),'b','LineWidth',1)
hold off



%% Plot Angular Velocity
figure
hold on
grid on
plot(Tspan,p,'r-','LineWidth',2)
plot(Tspan,q,'g-','LineWidth',2)
plot(Tspan,r,'b-','LineWidth',2)

legend('$\omega_x$','$\omega_y$','$\omega_z$','Interpreter','latex','FontSize',12)
xlabel('Time (sec)','FontSize',12)
ylabel('\omega (deg/s)','FontSize',12)

%% Plot Angular Velocity Splitted
figure
subplot(3,1,1)
hold on
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('deg/s','FontSize',12)
plot(Tspan,p,'r-','LineWidth',2)
legend('$\omega_x$','Interpreter','latex','FontSize',12')
hold off

subplot(3,1,2)
hold on
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('deg/s','FontSize',12)
plot(Tspan,q,'g-','LineWidth',2)
legend('$\omega_y$','Interpreter','latex','FontSize',12')
hold off

subplot(3,1,3)
hold on
grid on
xlabel('Time (sec)','FontSize',12)
ylabel('deg/s','FontSize',12)
plot(Tspan,r,'b-','LineWidth',2)
legend('$\omega_z$','Interpreter','latex','FontSize',12')
hold off
