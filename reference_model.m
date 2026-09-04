function dq_r = reference_model(q,qd,dqd,A)

q_bar = q - qd;

z = A*q_bar;

psi_z  = psi_sat(z);
%dpsi_z = psi_sat_derivative(z);

%dq_r  = dqd - psi_z;
dq_r = - psi_z; 

end