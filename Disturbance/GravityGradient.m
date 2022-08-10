%This function is used to Calculate the Gravity Gradient Torque
%G and Me are Earth constants
%Rc_vec is the vector between the origin of inertial frame and the CG of the satellite (3x1)
%I_mat is the inertia of the satellite
function [GTorque]=GravityGradient(G,Me,Rc,Rc_vec,I_mat)
GTorque=(3*G*Me/Rc^3).*(cross(Rc_vec,(I_mat.*Rc_vec)));
end