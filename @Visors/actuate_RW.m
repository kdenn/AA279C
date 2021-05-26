function M_act = actuate_RW(obj, M_des, w_sc, dt, add_noise)
% Purpose: Reaction wheel actuator modeling with added Gaussian noise

if ~exist('add_noise','var')
    add_noise = true;
end

L_dot_max = 0.0004; % (Nm)
N = numel(M_des);

% Get reaction wheel mounting matrix
A = obj.A_RW;
A_star = pinv(A);

% Command sent to actuator
L_dot_cmd = A_star * M_des;

if ~add_noise
    L_dot_act = L_dot_cmd;
    for i = 1:N
        L_dot_act(i) = sign(L_dot_act(i))*min(L_dot_max,abs(L_dot_act(i)));
    end
    M_act = A *L_dot_act;
    return
end

% Model Friction
% Wertz 7.9
N = numel(L_dot_cmd);

% Max torque
sn = sign(L_dot_cmd);
L_dot_act = L_dot_cmd;
for i = 1:N
    L_dot_act(i) = min(L_dot_max,abs(L_dot_act(i)));
end
% Each RW has an inertia of 0.0001147 kg-m^2 (calculated from m = 0.130 kg
% and V = 42x42x19 mm)
I = 0.0001147 * ones(4,1);
w_d = (sn.*L_dot_act./I)*dt + A_star * w_sc;
s = abs(60.*w_d./(2*pi)); % rpm
M_c = 0; % Coulomb friction coefficient (Nm)
f = 1.21E-6; % viscous friction coefficient (Nm/rpm)
M_f = M_c + f.*abs(s);
M_out = L_dot_act - M_f;
for i = 1:N
    L_dot_out(i,1) = sn(i)*max(0,M_out(i));
end

% 1-sigma noise for angular velocity
Q = (10^(-5))^2 * eye(1);
w_dot_noise = sqrtm(Q)*randn(4,1);

% Actual moment imparted on s/c
L_dot_noise = I .* w_dot_noise .* (L_dot_out~=0);
M_act = A * (L_dot_out + L_dot_noise);
end