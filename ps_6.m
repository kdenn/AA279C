% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 10; t_arr = (t0:dt:tf)';

% Visors class (not struct)
visors = Visors(w0, q0);

[omega_out, quat_out, rv_ECI_out, M_out] = visors.propagate(t_arr);

[m1_true, m2_true] = visors.get_ref_vecs_true(visors.ICs.JD_epoch);
[m1_meas, m2_meas] = visors.get_ref_vecs_meas(visors.ICs.JD_epoch, quat_out(:,1), visors.ICs.A_rot);
q_out = calc_q_det(m1_meas, m2_meas, m1_true, m2_true, visors.ICs.A_rot);

% m1_meas = get_m1_meas(visors.ICs.JD_epoch, quat_out(:,1), visors.ICs.A_rot);
% m2_meas = get_m2_meas(visors.ICs.JD_epoch, quat_out(:,1), visors.ICs.A_rot);
% m1_true = get_m1_true(visors.ICs.JD_epoch);
% m2_true = get_m2_true(visors.ICs.JD_epoch);
% q_out = calc_q_det(m1_meas, m2_meas, m1_true, m2_true, visors.ICs.A_rot);

function q_out = calc_q_det(m1_m, m2_m, m1_t, m2_t, A_rot)

p_m = m1_m;
q_m = cross(m1_m,m2_m) ./ norm(cross(m1_m,m2_m));
r_m = cross(p_m,q_m);
M = A_rot * [p_m, q_m, r_m];

p_v = m1_t;
q_v = cross(m1_t,m2_t) ./ norm(cross(m1_t,m2_t));
r_v = cross(p_v,q_v);
V = [p_v, q_v, r_v];

A = M * inv(V);

q_out = dcm2quat(A);
end

function m1_meas = get_m1_meas(JD, q, A)
m1_meas = get_m1_true(JD); 

% Get measurement in spacecraft body frame
m1_meas = A' * quat2dcm(q) * m1_meas;

% Noise characteristics
Q = 0.00 * eye(3);
mu = [0;0;0];
noise = sqrtm(Q)*randn(3,1) + mu;

% Corrupt measurement with noise
m1_meas = m1_meas + noise;
end

function m1_true = get_m1_true(JD)
% Cartesian coords of unit vector to Sun in ECI frame
m1_true = unitVec(get_sun_position(JD));
end

function m2_meas = get_m2_meas(JD, q, A)
m2_meas = get_m2_true(JD); 

% Get measurement in spacecraft body frame
m2_meas = A' * quat2dcm(q) * m2_meas;

% Noise characteristics
Q = 0.00 * eye(3);
mu = [0;0;0];
noise = sqrtm(Q)*randn(3,1) + mu;

% Corrupt measurement with noise
m2_meas = m2_meas + noise;
end

function m2_true = get_m2_true(JD)

% Right ascention of Alpha Centauri A in [HH, MM, SS]
ra = [14, 39, 35.06311];

% Declination of Alpha Centauri A in [deg, min, sec]
de = [-60, 50, 15.0992];

% Spherical coords
phi = ra(1) + ra(2)/60 + ra(2)/3600; 
phi = (phi/24)*2*pi;
theta = de(1) + sign(de(1))*(de(2)/60 + de(3)/3600);
theta = deg2rad(90 - theta);

% Cartesian coords of unit vector to Alpha Centauri A in ECI frame
m2_true = [cos(phi)*sin(theta); sin(phi)*sin(theta); cos(theta)];
end
