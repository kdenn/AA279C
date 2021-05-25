function M_act = actuate_RW(obj, L_dot_cmd)
% Purpose: Reaction wheel actuator modeling with added Gaussian noise

% Trisectrix
% A = [1, 0, 0, 1/sqrt(3); ...
%      0, 1, 0, 1/sqrt(3); ...
%      0, 0, 1, 1/sqrt(3)];
 
% Pyramid
A = (1/sqrt(3)) .* [-1, +1, +1, -1; ...
                    -1, -1, +1, +1; ...
                    +1, +1, +1, +1];
 
A_star = pinv(A);

% Command sent to actuator
L_dot_cmd = A_star * M_des;

% Model Friction
% Wertz 7.9
N = numel(L_dot_cmd);
r_wheel = 0.019/2;
s_max = 60 * (0.015 / (0.130 * r_wheel)) / r_wheel; % rpm
L_dot_max = 0.0004; % (Nm) 
sn = sign(L_dot_cmd);

X_dc = min(ones(N,1),max(0.125.*ones(N,1),L_dot_cmd./L_dot_max,2),2);
s = X_dc.*s_max;
M_c = 7.06E-4; % Coulomb friction coefficient (Nm)
f = 1.21E-6; % viscous friction coefficient (Nm/rpm)
M_f = M_c + f.*s;
L_dot_out = sn.*(X_dc.* L_dot_max - M_f);

% 1-sigma noise of 0.01 rad/s for angular velocity
% Each RW has an inertia of 0.0001147 kg-m^2 (calculated from m = 0.130 kg
% and V = 42x42x19 mm)
I = 0.0001147 * ones(4,1);
w0 = zeros(4,1);
Q = 0.01^2 * eye(1);
w_dot_noise = sqrtm(Q)*randn(4,1);

% Actual moment imparted on s/c
L_dot_noise = I .* (w0 + w_dot_noise);
M_act = A * (L_dot_out + L_dot_noise);
end