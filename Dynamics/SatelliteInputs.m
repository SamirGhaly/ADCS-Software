global w_O Inertia
%% Planet
Planet.Radius = 6378140e-3; %Km
Planet.Mass = 5.972e24; %Kg
Planet.Const = 6.6743e-20;  %Km^3/Kg/s^-2
Planet.mu = 3.986004418e5 ; %km^3.s^-2
%% 
Orbit.a=7046.1;                   %Semi Major Axis (km)
Orbit.incl=98.085;                     %inclination (deg)
Orbit.RAAN=301.643;                     %Right Ascension
Orbit.e=0;                        %Eccentricity       
Orbit.omega=291.1406;                     %Argument of Perigee
Orbit.TA=68.859;                        %True Anomaly v
Orbit.P=2*pi*sqrt(Orbit.a^3/Planet.mu);     % [sec] Orbit Period
w_O = 2*pi/Orbit.P;           % [rad/s] Orbit Angular Velocity
%%
%% Disturbance

%% Satellite
%Inertia and Mass
Sat.m = 1.2;   %Kg
Inertia = [0.06 0 0 ;0 0.08 0;0 0 0.004]; %Kg-m^2

%%%Intitial Conditions for Attitude and Angular Velocity
%Angles
IC.phi0 = 65; %deg
IC.theta0 =150 ;
IC.psi0 = -60;
%Omegas
IC.p0 = 10*pi/180;    %rad/s
IC.q0 = 10*pi/180;
IC.r0 = 10*pi/180;
