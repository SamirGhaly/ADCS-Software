%This Function is used to Calculate the magnetic disturbances on the satellite due to the permanent Magnetism of the Satellite
%B is the geocentric magnetic flux density (in Watt / m2)
%m_vec is spacecraft effective magnetic moment caused by permanent and induced magnetism and the spacecraft generated current loops
function MagTorque=MagneticDisturbance(m_vec,B)
MagTorque=m_vec.*B;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function is used to Calculate the Aerodynamic disturbances
%rho is  Atmospheric density
%V is norm of orbital velocity vector
%Cd is Drag coefficient
%N_vec is Outward normal unit vector for each satellite surface element
%V_vec is Unit vector in the direction of the translational velocity, V
%l is moment arm
function AeroTorque=AerodynamicDisturbance(rho,V_vec,Cd,N_vec,V,Area,l)
F_vec=0.5*Cd*rho*V^2*Area*dot(N_vec,V_vec).*V_vec;
AeroTorque=l.*F_vec;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function is used to Calculate the Solar disturbance
%I is energy per unit time through a cross-sectional unit area , in N/m^2
%c is speed of light, in m/sec 
%Crs is coefficient of specular reflection, the fraction of the incident radiation that is specularly reflected.
%Crd is coefficient of diffuse reflection, the fraction of the incident radiation that is diffusely reflected. 
%theta is is the angle between S_vec and N_vec
%N_vec is is Outward normal unit vector for each satellite surface element
%S_vec is the unit vector from the spacecraft to the Sun
%l is moment arm
function solTorque=SolarDisturbance(I,c,Crs,Crd,theta,N_vec,S_vec,Area,l)
F_vec=(-I/c).*(((1+Crs)*cos(theta)+(2/3)*Crd).*N_vec+(1-Crs)*sin(theta).*S_vec).*cos(theta).*Area;
solTorque=l.*F_vec;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function is used to Calculate the Gravity Gradient Torque
%G and Me are Earth constants
%Rc_vec is the vector between the origin of inertial frame and the CG of the satellite (3x1)
%I_mat is the inertia of the satellite
function GTorque=GravityGradient(G,Me,Rc,Rc_vec,I_mat)
GTorque=(3*G*Me/Rc^5).*Rc_vec.*(I_mat*Rc_vec);
end
