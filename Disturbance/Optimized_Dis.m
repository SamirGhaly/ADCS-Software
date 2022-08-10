function [F,T]=Optimized_Dis(Rc_vec,I_mat,S_vec,date,lat,lon,height,V_vec,B)
%% Constants 
t=0;
c=3e8;                        % speed of light
Crs=0.3;                        % coefficient of specular reflection, the fraction of the incident radiation that is specularly reflected.
Crd=0.3;                        % coefficient of diffuse reflection, the fraction of the incident radiation that is diffusely reflected. 
Fe=1367;
Area=1; 
Cd=2.2;                       % Drag coefficient
date = NewDate(t,date) ;
Parameters = DensityModelInitialize(date) ;
rho = msis86Modified(Parameters,date,lat,lon,height);
G=6.67428e-11;                % Earth constant       % e-11 or e-20
Me=5.97219e24;                % Mass of Earth 
m_vec=[0;0;0];
% Rc=6371e3;                    % Radius of Earth
%% Solar disturbance
%I is energy per unit time through a cross-sectional unit area , in N/m^2
%c is speed of light, in m/sec 
%Crs is coefficient of specular reflection, the fraction of the incident radiation that is specularly reflected.
%Crd is coefficient of diffuse reflection, the fraction of the incident radiation that is diffusely reflected. 
%theta is is the angle between S_vec and N_vec
%N_vec is is Outward normal unit vector for each satellite surface element
%S_vec is the unit vector from the spacecraft to the Sun (1x3)

N_mat=[1 0 0 ; 0 1 0 ; 0 0 1 ; -1 0 0 ; 0 -1 0; 0 0 -1];
R=[1 0 0 ; 0 1 0 ; 0 0 1 ; -1 0 0 ; 0 -1 0; 0 0 -1];
N=6;
p=Fe/c;
F_vec1=zeros(6,3);
T_vec1=zeros(6,3);
for i=1:N
    CosTheta = max(min(dot(N_mat(i,:),S_vec)/(norm(N_mat(i,:))*norm(S_vec)),1),-1);
    ThetaInDegrees = real(acosd(CosTheta));
    theta=ThetaInDegrees*(pi/180);
    F_vec1(i,:)=(-p).*((1-Crs).*S_vec+(2*Crs*cos(theta)+(1/3)*Crd).*N_mat(i,:))*cos(theta)*Area;
    T_vec1(i,:)=cross(R(i,:),F_vec1(i,:));
end
F1=F_vec1(1,:)+F_vec1(2,:)+F_vec1(3,:)+F_vec1(4,:)+F_vec1(5,:)+F_vec1(6,:);
T1=T_vec1(1,:)+T_vec1(2,:)+T_vec1(3,:)+T_vec1(4,:)+T_vec1(5,:)+T_vec1(6,:);
T1=T1';
%% Aerodynamic disturbances
%rho is  Atmospheric density
%V is norm of orbital velocity vector
%Cd is Drag coefficient
%N_vec is Outward normal unit vector for each satellite surface element
%V_vec is Unit vector in the direction of the translational velocity, V

N_mat=[1 0 0 ; 0 1 0 ; 0 0 1 ; -1 0 0 ; 0 -1 0; 0 0 -1];
R=[1 0 0 ; 0 1 0 ; 0 0 1 ; -1 0 0 ; 0 -1 0; 0 0 -1];
N=6;
F_vec2=zeros(6,3);
T_vec2=zeros(6,3);
omega=[0 0 2*pi/24/60/60];
r=Rc_vec;  % What is r ?
V_vec_rel=V_vec-cross(omega',r);   % do not forget to convert cross(omega,r) to body frame 

for i=1:N
F_vec2(i,:)=-0.5*Cd*rho*(norm(V_vec_rel))^2*Area*dot(N_mat(i,:),(V_vec_rel/norm(V_vec_rel))).*(V_vec_rel/norm(V_vec_rel));
if (dot(N_mat(i,:),V_vec_rel))<0
    F_vec2(i,:)=[0 0 0];
end
T_vec2(i,:)=cross(R(i,:),F_vec2(i,:));
end
F2=F_vec2(1,:)+F_vec2(2,:)+F_vec2(3,:)+F_vec2(4,:)+F_vec2(5,:)+F_vec2(6,:);
T2=T_vec2(1,:)+T_vec2(2,:)+T_vec2(3,:)+T_vec2(4,:)+T_vec2(5,:)+T_vec2(6,:);
T2=T2';
%% Gradient Torque
%G and Me are Earth constants
%Rc_vec is the vector between the origin of inertial frame and the CG of the satellite (3x1)
%I_mat is the inertia of the satellite

T_vec3=(3*G*Me/norm(Rc_vec)^5).*(cross(Rc_vec,(I_mat.*Rc_vec)));    %.* or *

%% Magnetic disturbances -------- on the satellite due to the permanent Magnetism of the Satellite
%B is the geocentric magnetic flux density (in Watt / m2)
%m_vec is spacecraft effective magnetic moment caused by permanent and induced magnetism and the spacecraft generated current loops
T_vec4=cross(m_vec,B);
%%
F=F1'+F2';
T=T1+T2+T_vec3+T_vec4;
end