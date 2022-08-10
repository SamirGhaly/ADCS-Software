close all
clear all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Aerodynamic disturbances
%rho is  Atmospheric density
%V is norm of orbital velocity vector
%Cd is Drag coefficient
%N_vec is Outward normal unit vector for each satellite surface element
%V_vec is Unit vector in the direction of the translational velocity, V
%l is moment arm

%% Solar disturbance
%c is speed of light, in m/sec 
%Crs is coefficient of specular reflection, the fraction of the incident radiation that is specularly reflected.
%Crd is coefficient of diffuse reflection, the fraction of the incident radiation that is diffusely reflected. 
%N_vec is is Outward normal unit vector for each satellite surface element
%S_vec is the unit vector from the spacecraft to the Sun (1x3)
%R is the vector from cg to the center of the surface ( Here is assumed cg=(0,0,0))

%% Gravity Gradient 
%G and Me are Earth constants
%Rc_vec is the vector between the origin of inertial frame and the CG of the satellite (3x1)
%I_mat is the inertia of the satellite

%% Magnetic disturbance
%B is the geocentric magnetic flux density (in Watt / m2)
%m_vec is spacecraft effective magnetic moment caused by permanent and induced magnetism and the spacecraft generated current loops

%% density
%lon is longitude [deg]  
%lat is  latitude [deg] 

%%
% G=6.67428e-11;                % Earth constant
% Me=5.97219e24;                % Mass of Earth 
% Rc=6371e3;                    % Radius of Earth 
Rc_vec=[1 1 1];               % vector between the origin of inertial frame and the CG of the satellite
I_mat=[0.9 0.9 0.9];          % the inertia of the satellite
% I=10000;                       % energy per unit time through a cross-sectional unit area , in N/m^2energy per unit time through a cross-sectional unit area , in N/m^2
% c=3e8;                        % speed of light
% Crs=0.3;                        % coefficient of specular reflection, the fraction of the incident radiation that is specularly reflected.
% Crd=0.3;                        % coefficient of diffuse reflection, the fraction of the incident radiation that is diffusely reflected. 
% Fe=1367;
% Area=1;                       
% l=0.1;                        % is the moment arm
lon =-82.485;                      % [deg]  (longitude)
lat  = 45.334;                   % [deg] (latitude)
height = 1944.3985;                 % [km]
i = 1 ;                       % ???
date = [2022 2 17 0 0 0] ;    
t = 0;                        
V_vec=[2.499 0 7.134]*1000;               % Unit vector in the direction of the translational velocity, V
% Cd=2.2;                       % Drag coefficient
V=1;                         % Velocity
m_vec=[ 1 1 1 ]*0.0006;           % spacecraft effective magnetic moment caused by permanent and induced magnetism and the spacecraft generated current loops
B=[30 30 30];                 % the geocentric magnetic flux density (in Watt / m2)
datee = NewDate(t,date) ;
Parameters = DensityModelInitialize(datee) ;
rho = msis86Modified(Parameters,date,lat,lon,height);
JD = JDate(date);
SS=Sun_Pos( JD);
phi_theta_psi=[30;40;50]*pi/180;
q0123_0=EulerAngles2Quaternions(phi_theta_psi);
SS_body=newfrm(SS,q0123_0);
S_vec=transpose(SS_body);                   % the unit vector from the spacecraft to the Sun
S_vec_normalized=S_vec/norm(S_vec) ;
tic
[F,T]=AttitudeDisturbance(Rc_vec,I_mat,S_vec,date,t,lat,lon,height,V_vec,V,m_vec,B)
toc
tic
[F,T]=Optimized_Dis(Rc_vec,I_mat,S_vec,date,t,lat,lon,height,V_vec,V,m_vec,B)
toc
%[F T]=SolarDisturbance(S_vec_normalized)
%[F,T]=AerodynamicDisturbance(date,t,lat,lon,height,V_vec,V)
% GravityGradient(Rc_vec,I_mat)
%MagneticDisturbance(m_vec,B)