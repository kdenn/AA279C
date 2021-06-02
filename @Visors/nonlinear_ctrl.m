function M_c = nonlinear_ctrl(obj, q_des, q_est, w_des, w_est)

% Calculation of gains Kp and Kd
f = 10 * obj.ICs.n;
zeta = 0.7;
Kp = (f^2) .* diag(obj.ICs.I_princ);
Kd = 2 .* zeta .* f .* diag(obj.ICs.I_princ);

% Kp = (f^2) ./ diag(obj.ICs.I_princ);
% Kd = zeros(3,1);
% Ip = diag(obj.ICs.I_princ);
% dI = [Ip(3)-Ip(2);
%       Ip(1)-Ip(3);
%       Ip(2)-Ip(1)];
% for i = 1:3
%     Kd(i) = 2*sqrt(Ip(i)*(3*(obj.ICs.n^2)*dI(i)+Kp(i)));
% end

% Attitude tracking error DCM
As = quat2dcm(q_est);
At = quat2dcm(q_des);
Ae = As * At';

% Output desired control torque
[x_couple, y_couple, z_couple] = feedback_linearization(w_est(1),w_est(2),w_est(3),obj.ICs.Ix,obj.ICs.Iy,obj.ICs.Iz);
c = [x_couple;y_couple;z_couple];
M_c = zeros(3,1);
for i=1:3
    j = mod(i+1,3); if j == 0 j = 3; end
    k = mod(i+2,3); if k == 0 k = 3; end
    
    alpha_i = (Ae(j,k) - Ae(k,j)) / 2;
    M_c(i) = - Kp(i)*alpha_i - Kd(i)*(w_est(i)-w_des(i));
end
end

function [x_couple, y_couple, z_couple] = feedback_linearization(wx_meas, wy_meas,wz_meas,Ix,Iy,Iz)
    x_couple=(Iz-Iy)*wy_meas*wz_meas;
    y_couple=(Ix-Iz)*wx_meas*wz_meas;
    z_couple=(Iy-Ix)*wy_meas*wx_meas;
end