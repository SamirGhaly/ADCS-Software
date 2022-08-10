%This Function is used to Calculate the magnetic disturbances on the satellite due to the permanent Magnetism of the Satellite
%B is the geocentric magnetic flux density (in Watt / m2)
%m_vec is spacecraft effective magnetic moment caused by permanent and induced magnetism and the spacecraft generated current loops
function [MagTorque]=MagneticDisturbance(m_vec,B)
MagTorque=cross(m_vec,B);
end