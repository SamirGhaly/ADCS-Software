function [mu,Tdes]=control(delta_q,omega,Control_Method,q_desired,B,q_B_O)

if Control_Method==0  %No Controller
    Tdes=[0;0;0];
    mu=[0;0;0];
elseif Control_Method==1  %PD Controller
    [mu,Tdes]=PD(delta_q,omega,B,q_B_O);

elseif Control_Method==2  %Sliding Mode Controller
    [mu,Tdes]=SMC(delta_q,omega,q_desired,B);

elseif Control_Method==3 %Bdot
    [mu,Tdes]=B_dot_Control(B,omega);

elseif Control_Method==4  %K Omega
    Kp=0.005;
    Tdes=-Kp*omega;
    mu=cross(B,Tdes)/norm(B)^2;
    
elseif Control_Method==5   %LQR
    mu=LQR(delta_q,omega,B);
    Tdes=[0;0;0];
end

end