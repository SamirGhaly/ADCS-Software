clc
clear all
close all
format longG
global  Orbit dt Control_Method q_desired M_max Parameters date Inertia Bx_ECI By_ECI Bz_ECI Bx_Body By_Body Bz_Body Bx_ECEF By_ECEF Bz_ECEF Bx_Orbit By_Orbit Bz_Orbit
global  date w_O q_desired CTRL

%SatelliteInputs_Sharma2021
SatInp

addpath('MSIS86\');

%Simulation Parameters
tfinal=6000;
dt =1;
Tspan = 0:dt:tfinal;
date = [2022 07 10 0 0 0];
Date_Str=datestr(date);
%% Control
%Desired Attitude
Angles_Desired=[10;5;-5]*pi/180;
q_desired=EulerAngles2Quaternions(Angles_Desired);

%Controller
Control_Algorithm('PD');    %No Controller / PD / SMC / Bdot / K Omega / LQR
%M_max=2;           %Max Dipole Moment
M_max=0.103;

% Bdot Gains
%CTRL.K=1e6;
CTRL.K = [15000;15000;12000];

% PD Gains
CTRL.Kp=[7.70370370370371e-09 0 0;0 7.70370370370371e-09  0 ;0 0 7.70370370370371e-09]*2;
CTRL.Kd=[5.77777777777779e-06  0 0;0 5.77777777777779e-06  0 ;0 0 5.77777777777779e-06 ]*1.5*1.25;


% Controller Panel
CTRL.SWITCH  = 6000; % Time to change mode from Detumbling to Pointing.

% t=0;
% datee = NewDate(t,date) ;
% Parameters = DensityModelInitialize(datee);
tic
stateout=Dynamics_Kinematics(Orbit,IC,Tspan,dt);
toc
%% Post Processing
Simulation

%% Export to Excel
%filename='results_NoController.xlsx';
%Results2XLSX;
%winopen(filename)