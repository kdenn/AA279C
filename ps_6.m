% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 10; t_arr = (t0:dt:tf)';

% Set options for class
opts = struct();
opts.calc_q_det_flag = 1;
opts.corrupts_measurements = 1;

% Visors class (not struct)
visors = Visors(w0, q0, opts);

[omega_out, quat_out, rv_ECI_out, M_out] = visors.propagate(t_arr);

[m1_true, m2_true] = visors.get_ref_vecs_true(visors.ICs.JD_epoch);
[m1_meas, m2_meas] = visors.get_ref_vecs_meas(visors.ICs.JD_epoch, quat_out(:,1));
q_out = visors.calc_q_stat(m1_meas, m2_meas, m1_true, m2_true);

q_out - q0