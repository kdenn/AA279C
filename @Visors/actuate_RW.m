function M_act = actuate_RW(obj, M_des, add_noise)
% Purpose: Reaction wheel actuator modeling with added Gaussian noise

if ~exist('add_noise','var')
    add_noise = true;
end

L_dot_max = 0.0004; % (Nm)
N = numel(M_des);

if ~add_noise
    M_act = M_des;
    for i = 1:N
        M_act(i) = sign(M_act(i))*min(L_dot_max,abs(M_act(i)));
    end
    return
end

% Get reaction wheel mounting matrix
A = obj.A_RW;
A_star = pinv(A);

% Command sent to actuator
L_dot_cmd = A_star * M_des;

% Model Friction
% Wertz 7.9
N = numel(L_dot_cmd);
r_wheel = 0.019/2;
s_max = 60 * (0.015 / (0.130 * r_wheel)) / (2*pi*r_wheel); % rpm
sn = sign(L_dot_cmd);

X_min = 0; %0.125;
X_dc = nan(N,1);
for i = 1:N
    X_dc(i) = min(1,max(X_min,abs(L_dot_cmd(i)/L_dot_max)));
end
s = X_dc.*s_max;
M_c = 7.06E-4; % Coulomb friction coefficient (Nm)
f = 1.21E-6; % viscous friction coefficient (Nm/rpm)
M_f = M_c + f.*s;
M_out = X_dc.* L_dot_max - M_f;
for i = 1:N
    L_dot_out(i,1) = sn(i)*max(0,M_out(i));
end

% 1-sigma noise of 0.01 rad/s for angular velocity
% Each RW has an inertia of 0.0001147 kg-m^2 (calculated from m = 0.130 kg
% and V = 42x42x19 mm)
I = 0.0001147 * ones(4,1);
w0 = zeros(4,1);
Q = 0.01^2 * eye(1);
w_dot_noise = sqrtm(Q)*randn(4,1);

% Actual moment imparted on s/c
L_dot_noise = I .* (w0 + w_dot_noise) .* (L_dot_out~=0);
M_act = A * (L_dot_out + L_dot_noise);
end