global ERR Kalman Mag Gyro

ERR.P=[(100e-1)*eye(4,4)    zeros(4,3);
         zeros(3,4)         100*eye(3)];
quat_sigma=[1; 1;1;1 ]*5e-2;
omega_sigma=[1;1;1]*8.727e-4;
R(1,1)=quat_sigma(1);R(2,2)=quat_sigma(2);R(3,3)=quat_sigma(3);R(4,4)=quat_sigma(4);
R(5,5)=omega_sigma(1);R(6,6)=omega_sigma(2);R(7,7)=omega_sigma(3);
ERR.R=R;

Mag.A_magnetometer=eye(3);
sigma_quat=5e-8; 
Mag.n_magnetometer=normrnd(0,sigma_quat,[3,1]);

Gyro.A_gyro=eye(3);
sigma_omega=8.727e-4; 
% sigma_omega=5e-8; 
system_noise=1e-5;
Gyro.n_omega=normrnd(0,sigma_omega,[3,1]);
Kalman.S=system_noise^2*eye(3,3);
Kalman.Q1=[zeros(4,4) zeros(4,3);zeros(3,4) Kalman.S]    ;