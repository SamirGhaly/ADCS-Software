function output=Magnetometer(input)
global Mag
% Actual State
B_Body=input(1:3);

B_Body_measured=B_Body+Mag.n_magnetometer;

output=B_Body_measured;
end