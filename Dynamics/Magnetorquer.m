function [LMN_mgtorquers,m_vec] = Magnetorquer(B_D,Pulses,mu_des,M_max)

% M_max max moment by manufacturer
% B_D magnetic feild w.r.t Device ( Magnetorquer ) /or Body if Device is in
% the same orientation
% Pulses = n_vec/N (from controller), N = max pulses
% tdes/tmax

m_vec=Pulses*M_max; 
%m_vec = min(abs(mu_des),M_max).*sign(mu_des) ;
LMN_mgtorquers=cross(m_vec,B_D);

%M=cross(B_D,LMN_mgtorquers)/norm(B_D)^2;               %Dipole Moment [where B_D is the estimated]

end