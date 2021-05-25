function M_c = nonlinear_ctrl(obj, q_des, q_est, w_des, w_est)

% Calculation of gains Kp and Kd
f = 10 * obj.ICs.n;
zeta = 0.7;
Kp = diag(obj.ICs.I_princ) .* (f^2);
Kd = diag(obj.ICs.I_princ) .* 2 .* zeta .* Kp;

% Attitude tracking error DCM
As = quat2dcm(q_est);
At = quat2dcm(q_des);
Ae = As * At';

% Output desired control torque
M_c = zeros(3,1);
for i=1:3
    j = mod(i+1,3); if j == 0 j = 3; end
    k = mod(i+2,3); if k == 0 k = 3; end
    
    alpha_i = (Ae(j,k) - Ae(k,j)) / 2;
    M_c(i) = - Kp(i)*alpha_i - Kd(i)*(w_est(i)-w_des(i));
end
end