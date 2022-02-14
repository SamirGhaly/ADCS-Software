%%%Initialize
clear
clc
close all
%%%Globals
global  m I invI
%%%%Simulation of a Low Earth Satellite
disp('Simulation Started')
%%%Get Planet Parameters
planet
%%%Get mass and inertia properties
inertia
%%%Initial Conditions Position and Velocity
altitude = 600*1000; %%meters
x0 = R + altitude;
y0 = 0;
z0 = 0; 
xdot0 = 0;
inclination = 56*pi/180;
semi_major = norm([x0;y0;z0]);
vcircular = sqrt(mu/semi_major);
ydot0 = vcircular*cos(inclination);
zdot0 = vcircular*sin(inclination);
%%%Intitial Conditions for Attitude and Angular Velocity
phi0 = 0;
theta0 = 0;
psi0 = 0;
ptp0 = [phi0;theta0;psi0];
q0123_0 = EulerAngles2Quaternions(ptp0);
p0 = 1;
q0 = 0;
r0 = 0;
state = [x0;y0;z0;xdot0;ydot0;zdot0;q0123_0;p0;q0;r0];
period = 2*pi/sqrt(mu)*semi_major^(3/2);
tspan = [0 period];
period = 2*pi/sqrt(mu)*semi_major^(3/2);
number_of_orbits = 1;
tfinal = period;
timestep = 1.0;
tout = 0:timestep:tfinal;
stateout = zeros(length(tout),length(state))
for idx = 1:length(tout)
    %%%Save the current state
    stateout(idx,:) = state';
  
    %%%%Then we make our 4 function calls for the RK4
    k1 = Satellite(tout(idx),state);
    k2 = Satellite(tout(idx)+timestep/2,state+k1*timestep/2);
    k3 = Satellite(tout(idx)+timestep/2,state+k2*timestep/2);
    k4 = Satellite(tout(idx)+timestep,state+k3*timestep);
    k = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    state = state + k*timestep;
  
end
disp('Simulation Complete')
%%%This is where we integrate the equations of motion
disp('Simulation Complete')
%%%Convert state to kilometers
stateout(:,1:6) = stateout(:,1:6)/1000;
%%%Extract the state vector
xout = stateout(:,1);
yout = stateout(:,2);
zout = stateout(:,3);
q0123out = stateout(:,7:10);
ptpout = Quaternions2EulerAngles(q0123out);
pqrout = stateout(:,11:13);
%%%Make an Earth
[X,Y,Z] = sphere(100);
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000;
%%%Plot X,Y,Z as a function of time
fig0 = figure();
set(fig0,'color','white')
plot(tout,xout,'b-','LineWidth',2)
hold on
grid on
plot(tout,yout,'r-','LineWidth',2)
plot(tout,zout,'g-','LineWidth',2)
xlabel('Time (sec)')
ylabel('Position (m)')
legend('X','Y','Z')

%%%Plot 3D orbit
fig = figure();
set(fig,'color','white')
plot3(xout,yout,zout,'b-','LineWidth',4)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold on
surf(X,Y,Z,'EdgeColor','none')
axis equal
%%%plot Euler Angles
fig4 = figure();
set(fig4,'color','white')
plot(tout,ptpout,'LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Angles (rad)')
%%%Plot Angular Velocity
fig5 = figure();
set(fig5,'color','white')
plot(tout,pqrout,'LineWidth',2)
grid on
xlabel('Time (sec)')
ylabel('Angular Velocity (rad/s)')