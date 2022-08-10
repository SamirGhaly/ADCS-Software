function delta_q=q_error(q_actual,q_desired)

q_desired_inv=conjq(q_desired)/norm(q_desired)^2;

delta_q=quatmul(q_actual,q_desired_inv);

end