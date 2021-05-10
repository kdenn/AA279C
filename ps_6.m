% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 60*30; t_arr = (t0:dt:tf)';

% Set options for class
opts = struct();
opts.calc_q_det_flag = 1; % 1 if fictitious meas, 0 if normal meas
opts.corrupt_measurements = 1; % 1 if introducing noise to meas

% Visors class
vsrs = Visors(w0, q0, opts);
vsrs.opts.est_q = @vsrs.calc_q_det; % deterministic or statistical?

% Propagate
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr);

%%
plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q);
plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q_from_w);
plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q);
plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q_from_w);
