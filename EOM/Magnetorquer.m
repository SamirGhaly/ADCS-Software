%% Magnetorquer Device
% M_max max moment by manufacturer
% B_D magnetic feild w.r.t Device ( Magnetorquer )
% (n_vec/N) from controller
function LMN_mgtorquers = Magnetorquer(B_D,m_vec,n_vec,N,M_max)
m_vec=(n_vec/N)*M_max;
LMN_mgtorquers=cross(B_D,m_vec);
end