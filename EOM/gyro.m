function w_D=gyro(w_B,q_BD)  %Omega calculated in Body w.r.t. Inertia
%Noise
sigma=0.5; % Device Data 
s=normrnd(0,sigma,[3,1]);
zeta=s;
%Omega in Device w.r.t Body
w_D=newfrm(w_B,q_BD)+zeta;
end