function output=Gyro(input)
global Gyro
% Actual State
w_B_I_actual=input(1:3);

w_d=Gyro.A_gyro*w_B_I_actual+Gyro.n_omega;
w_obs=inv(Gyro.A_gyro)*w_d;

output=w_obs;
end