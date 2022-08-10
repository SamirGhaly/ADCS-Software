function output=EKF(input)
global dt Kalman ERR CONST Inertia Angles0
% Estimated States
q_B_O=input(1:4);
w_B_I=input(5:7);

p=w_B_I(1);
q=w_B_I(2);
r=w_B_I(3);

q1=q_B_O(1);
q2=q_B_O(2);
q3=q_B_O(3);
q4=q_B_O(4);

% Sensor Data
w_obs=input(8:10);
B_Body_measured=input(11:13);

% Magnetic Field
B_Orbit=input(14:16);
% Estimation Algorithm

[state_transition,F]=state_trans(Inertia,p,q,r,CONST.w_O,q1,q2,q3,q4,dt);
F12=F(1:4,5:7);
F22=F(5:7,5:7);
MM=F12*Kalman.S;
Q2=[zeros(4,4) MM;(F12*Kalman.S)'  F22'*Kalman.S+F22*Kalman.S];
Q3=[F12*Kalman.S*F12' F12*Kalman.S*F22';F22*Kalman.S*F12' F22*Kalman.S*F22'];
Q=Kalman.Q1*dt+Q2*(dt)^2/2+Q3*(dt)^3/3;

%Predicted Covarience
P_pred=state_transition*ERR.P*state_transition'+Q;

%Extract Quaternion from Magnetic Field
x0 = Angles0;
fun = @(x)getDCM(x,B_Orbit,B_Body_measured);
Angles_corr=wrapTo2Pi(fsolve(fun,x0));
q_obs=eul2q(Angles_corr,'xyz','wxyz');

%   q_obs=A_magnetometer*state(7:10)+n_magnetometer;

z_obs=[q_obs; w_obs];

%Observation Equation H
H=eye(7);

%Kalman Gain
SS=H*P_pred*H'+ERR.R;
K=P_pred*(H')*inv(SS);

%get corrected state
state=input(1:7);
state_corr=state+K*(z_obs-state);

%update Error Covariance
ERR.P=(eye(7)-K*H)*P_pred;

q_B_O_Corrected=state_corr(1:4);
w_B_I_Corrected=state_corr(5:7);


output=[q_B_O_Corrected;w_B_I_Corrected];

end
