function M_act = actuate_RW(obj, M_des)
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

% Each RW has an inertia of 0.0001147 kg-m^2 (calculated from m = 0.130 kg
% and V = 42x42x19 mm)
I = 0.0001147 * ones(4,1);
w0 = zeros(4,1);

% Command sent to actuator
L_dot_cmd = A_star * M_des;
w_dot_cmd = (L_dot_cmd ./ I) + w0; 

% 1-sigma noise of 0.01 rad/s for angular velocity
Q = 0.01^2 * eye(1);
w_dot_act = max(0, w_dot_cmd + sqrtm(Q)*randn(4,1));

% Actual moment imparted on s/c
L_dot_act = I .* (w0 + w_dot_act);
M_act = A * L_dot_act;
end