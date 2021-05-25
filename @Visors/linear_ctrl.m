function M_c = linear_ctrl(obj, q_des, q_est, w_des, w_est)

% Calculation of gains Kp and Kd
f = 10 * obj.ICs.n;
zeta = 0.7;
Kp = diag(obj.ICs.I_princ) .* (f^2);
Kd = diag(obj.ICs.I_princ) .* 2 .* zeta .* Kp;

% Attitude tracking error DCM
a_est = dcm2eulerangles(quat2dcm(q_est));
a_des = dcm2eulerangles(quat2dcm(q_des));
alpha = a_des - a_est;
alpha_d = w_des - w_est;

% Output desired control torque
M_c = - Kp .* alpha - Kd .* alpha_d;

end