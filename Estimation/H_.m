function H=H_(BO,q0123_est)
q_s_est=q0123_est(1);
q_v_est=q0123_est(2:4);
B_est=BO+2*q_s_est*cross(BO,q_v_est)-2*cross(cross(q_v_est,BO),q_v_est);
H=[-2*C_cross(B_est) zeros(3,3);zeros(3,3) eye(3,3)];
end