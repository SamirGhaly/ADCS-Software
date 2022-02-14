clc
clear all
clear vars
phi_theta_psi=[30 50 40];
q7=EulerAngles2Quaternions(phi_theta_psi)
q9=ConvertRep(phi_theta_psi,'E','Q')

q_BD=q9;
w_B=[10 20 30];
w_D=gyro(w_B,q_BD)
B_E=1e4*[2.0641 -0.1757 -1.0079];
% q_EB=quaternion(q7)
% q_BD=1.5*quaternion(q7)
q_EB=q9
q_BD=1.5*q9
B_D=magnometer(B_E,q_EB,q_BD)
%B_D=

% [a,b,c,d]=parts(q_EB);
% [e,f,g,h]=parts(q_BD);
% q1=[a b c d];
% q2=[e f g h];
% 
% q3=quatmul(q1,q2)
% q=q_EB*q_BD

