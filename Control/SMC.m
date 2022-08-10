function [mu,L]=SMC(delta_q,omega,q_desired,B_Body)
global Inertia

desired_rates=[0;0;0];

eular_c=Quaternions2EulerAngles(q_desired);
phi_c=eular_c(1);
theta_c=eular_c(2);
epsi_c=eular_c(3);


phi_dot=desired_rates(1);
theta_dot=desired_rates(2);
epsi_dot=desired_rates(3);



omega_c=[epsi_dot-phi_dot*sin(theta_c);
    phi_dot*cos(theta_c)*sin(epsi_c)+theta_dot*cos(epsi_c);
    phi_dot*cos(theta_c)*cos(epsi_c)-theta_dot*sin(epsi_c)];


omega_dot_c=epsi_dot*[0;
    phi_dot*cos(theta_c)*cos(epsi_c)-theta_dot*sin(epsi_c);
    -phi_dot*cos(theta_c)*sin(epsi_c)-theta_dot*cos(epsi_c)];



% K=0.015;        
% G=0.15*eye(3);

K=0.0000015;        %Test
G=0.1*eye(3);  %Test

delta_q1=delta_q(1);   %Scalar Part
delta_q24=delta_q(2:4);  %Vector Part
s=omega-omega_c+K*sign(delta_q1)*delta_q24;

e=0.01;
s_bar=zeros(3,1);
for i=1:3
    if s(i)>e
        s_bar(i)=1;
    elseif abs(s(i))<= e
        s_bar(i)= s(i)/e;
    elseif s(i)< -e
        s_bar(i)=-1;
    end
end

L=Inertia*((K/2)*(abs(delta_q1)*(omega_c-omega)-cross(sign(delta_q1)*delta_q24,(omega+omega_c)))+...
    omega_dot_c-G*s_bar);

mu=cross(B_Body,L)/norm(B_Body)^2;


end