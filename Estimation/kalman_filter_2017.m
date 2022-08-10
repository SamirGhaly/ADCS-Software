function statecorr =kalman_filter_2017(q0123,BO,q0123_a,w_B,w_est,Kk)
    q_BD=[1;0;0;0];
    zeta=normrnd(0,5e-7,[3,1]);
%      H=H_(BO,q0123);
%      Kk=kalman_gain(P_pred,H,R_t);
     B_est=newfrm(BO,q0123);
     B_bb=newfrm(BO,q0123_a)+zeta ;        
     y_m(1:3)=B_est/norm(B_est)-B_bb/norm(B_bb);
%      w_B=stateout(11:13,idx);
%       w_est=statecorr(11:13,idx);
     y_m(4:6)=w_est-gyro(w_B,q_BD);
     dx_corr=Kk*y_m';
     M=dx_corr(1:3);
     dw=dx_corr(4:6);
     M0=sqrt(1-norm(M));
     M_final=[M0 ;-M];
     q_pred=q0123;
     q_corr=quatmul(q_pred,M_final);
     pqr_corr=w_est-dw;
%      q_est=q_corr(2:4);
     statecorr=[q_corr;pqr_corr];%output
    
end