function [u_I,v_I,w_I] =O2I(u_o,v_o,w_o,beta0,w0,t) %Orbital to Earth Inertial
beta=beta0+w0*t;
%beta is latitude position of satalite
%t 
I2O = [cosd(beta) 0 -sind(beta);
          0      -1  0;
       -sind(beta) 0  cosd(beta)];
V_I=inv(I2O)*[u_o,v_o,w_o];
u_I=V_I(1);v_I=V_I(2);w_I=V_I(3);
end