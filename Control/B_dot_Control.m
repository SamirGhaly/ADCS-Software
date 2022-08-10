function [mu,Tourqe]=B_dot_Control(B_Body,pqr)
global dt CTRL
persistent B_old

if isempty(B_old)
    B_old = [0;0;0] ;
end
B_dot = (B_Body - B_old)/dt ;
%B_dot = cross(B_Body,pqr);
B_old=B_Body;

mu=-CTRL.K .*B_dot;
Tourqe = cross(mu,B_Body);
end