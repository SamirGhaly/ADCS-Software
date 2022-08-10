function [q_O_I]=I2O(r,v)
k_=-r/norm(r);
j_=cross(v,r)/norm(cross(v,r));
i_=cross(j_,k_);

%quaternion from Inertial to Orbit
dcm_i2o=([i_';j_';k_']);
q_O_I=dcm2quat(dcm_i2o); 
end