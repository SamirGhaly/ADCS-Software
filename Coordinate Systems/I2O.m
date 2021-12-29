function [u_o,v_o,w_o] =I2O(u_I,v_I,w_I,beta0,w0,t) %Earth Inertial to Orbital

beta=beta0+w0*t;
%beta is latitude position of satalite
%t 
I2O = [cosd(beta) 0 -sind(beta);
          0      -1  0;
       -sind(beta) 0  cosd(beta)];
V_o=I2O*[u_I,v_I,w_I];
u_o=V_o(1);v_o=V_o(2);w_o=V_o(3);
end