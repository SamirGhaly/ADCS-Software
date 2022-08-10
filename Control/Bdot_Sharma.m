function L=Bdot_Sharma(B_Body,pqr)
global mu
% Sharma 2021 Paper 

k=1.0405e-03;
mu=-k*cross(B_Body,pqr)/norm(B_Body)^2;
mu = min(2, max(-2, mu));  %Saturation

L=cross(mu,B_Body);

end