function y = g(x)
% Purpose: calculate y = g(x,u)

% Renaming variables
wx = x(1); wy = x(2); wz = x(3);
q1 = x(4); q2 = x(5); q3 = x(6); q4 = x(7); q_vec = [q1; q2; q3; q4];

% Get ECI directions to Sirius and Alpha Centauri
[Sir_ECI, Alp_ECI] = Visors.get_ref_vecs_true();

% Get inertia properties
visorsInertia;

% Get expected measurements in spacecraft body frame
Sir_meas = A_rot' * quat2dcm(q_vec) * Sir_ECI;
Alp_meas = A_rot' * quat2dcm(q_vec) * Alp_ECI;

% Expected output measurement vector (9x1)
y = zeros(9,1);
y(1) = wx;
y(2) = wy;
y(3) = wz;
y(4:6) = Sir_meas;
y(7:9) = Alp_meas;
end