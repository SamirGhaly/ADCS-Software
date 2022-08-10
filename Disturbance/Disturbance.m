%This function is used to Calculate the Aerodynamic disturbances
%rho is  Atmospheric density
%V is norm of orbital velocity vector
%Cd is Drag coefficient
%N_vec is Outward normal unit vector for each satellite surface element
%V_vec is Unit vector in the direction of the translational velocity, V
%l is moment arm
function [F_D,M_D] =Disturbance() 

function [AeroTorque,F1]=AerodynamicDisturbance(date,t,lat,lon,height,V_vec,Cd,V,Area,l)
date = NewDate(t,date) ;
Parameters = DensityModelInitialize(date) ;
rho = msis86Modified(Parameters,date,lat,lon,height);
N_mat=[1 0 0 ; 0 1 0 ; 0 0 1 ; -1 0 0 ; 0 -1 0; 0 0 -1];
N=6;
F_vec=zeros(6,3);
for i=1:N
F_vec(i,:)=0.5*Cd*rho*V^2*Area*cross(dot(N_mat(i,:),V_vec).*N_mat(i,:),V_vec);
end
F1=F_vec(1,:)+F_vec(2,:)+F_vec(3,:)+F_vec(4,:)+F_vec(5,:)+F_vec(6,:);
AeroTorque=l.*F1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function is used to Calculate the Solar disturbance
%I is energy per unit time through a cross-sectional unit area , in N/m^2
%c is speed of light, in m/sec 
%Crs is coefficient of specular reflection, the fraction of the incident radiation that is specularly reflected.
%Crd is coefficient of diffuse reflection, the fraction of the incident radiation that is diffusely reflected. 
%theta is is the angle between S_vec and N_vec
%N_vec is is Outward normal unit vector for each satellite surface element
%S_vec is the unit vector from the spacecraft to the Sun (1x3)
%l is moment arm
function [solTorque,F2]=SolarDisturbance(I,c,Crs,Crd,theta,S_vec,Area,l)
N_mat=[1 0 0 ; 0 1 0 ; 0 0 1 ; -1 0 0 ; 0 -1 0; 0 0 -1];
N=6;
F_vec=zeros(6,3);
for i=1:N
F_vec(i,:)=(-I/c).*(((1+Crs)*cos(theta)+(2/3)*Crd).*N_mat(i,:)+(1-Crs)*sin(theta).*S_vec).*cos(theta).*Area;
end
F2=F_vec(1,:)+F_vec(2,:)+F_vec(3,:)+F_vec(4,:)+F_vec(5,:)+F_vec(6,:);
solTorque=l.*F2;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function is used to Calculate the Gravity Gradient Torque
%G and Me are Earth constants
%Rc_vec is the vector between the origin of inertial frame and the CG of the satellite (3x1)
%I_mat is the inertia of the satellite
function GTorque=GravityGradient(G,Me,Rc,Rc_vec,I_mat)
GTorque=(3*G*Me/Rc^5).*Rc_vec.*(I_mat*Rc_vec);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This Function is used to Calculate the magnetic disturbances on the satellite due to the permanent Magnetism of the Satellite
%B is the geocentric magnetic flux density (in Watt / m2)
%m_vec is spacecraft effective magnetic moment caused by permanent and induced magnetism and the spacecraft generated current loops
function MagTorque=MagneticDisturbance(m_vec,B)
MagTorque=cross(m_vec,B);
end
F_D=F1+F2;
M_D=GTorque+MagTorque+AeroTorque+solTorque;
end