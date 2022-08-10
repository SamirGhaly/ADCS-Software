function B_D=magnometer(B_E,q_EB,q_BD)
%Noise
sigma=1; % Device Data 
s=normrnd(0,sigma,[3,1]);
zeta=s;


q_ED=quatmul(q_EB,q_BD);
B_D=newfrm(B_E,q_ED)+zeta;

end

















