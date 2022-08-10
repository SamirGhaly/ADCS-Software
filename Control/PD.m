function [mu,T_des]=PD(delta_q,w_B_BI,B_Body,q_B_O)
global w_O CTRL
delta_q_v=delta_q(2:4);               %vector parts

w_O_OI=[0;-w_O;0];                %Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
A_q_O_B=ATT(q_B_O);
w_B_OI=A_q_O_B*w_O_OI;
delta_omega=w_B_BI-w_B_OI;             %w_B_BO

%% Control Law
T_des=-CTRL.Kp*delta_q_v-CTRL.Kd*delta_omega;

mu=cross(B_Body,T_des)/norm(B_Body)^2;
%mu=-(kp/norm(B_Body,2)^2)*cross(B_Body,delta_q_v)-(kd/norm(B_Body,2)^2)*cross(B_Body,delta_omega);
%% PD [SIDI Ch.7]
% T_des=2*kp*delta_q(1)*delta_q_v + kd*delta_omega;
% mu=cross(B_Body,T_des)/norm(B_Body)^2;
end
