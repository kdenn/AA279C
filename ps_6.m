% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 60*30; t_arr = (t0:dt:tf)';

add_noise = 1;

% Visors class
vsrs = Visors(w0, q0);

q_des = dcm2quat(vsrs.ICs.A_rot); % body/ECI -> princ
q_des = repmat(q_des,[1,numel(t_arr)]);

%% Part 3a: deterministic

% Options
vsrs.opts.calc_q_det_flag = 0; % 1 if fictitious meas, 0 if normal meas
vsrs.opts.corrupt_measurements = add_noise; % 1 if introducing noise to meas
vsrs.opts.est_q = @vsrs.calc_q_det; % deterministic or statistical?

% Propagate
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr);

% Plots
% plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q); % 7.3.1.a
% plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q); % 7.3.1.b
plot_q_est_vs_q_true(t_arr, q_des, vsrs.est.q); % control error
plot_q_est_vs_q_true_diff(t_arr, q_des, vsrs.est.q); % control error

%% Part 3a: deterministic variant
% Visors class
vsrs = Visors(w0, q0);

% Options
vsrs.opts.calc_q_det_flag = 1; % 1 if fictitious meas, 0 if normal meas
vsrs.opts.corrupt_measurements = add_noise; % 1 if introducing noise to meas
vsrs.opts.est_q = @vsrs.calc_q_det; % deterministic or statistical?

% Propagate
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr);

% Plots
%plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q); %3
% plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q); %4 7.3.2
plot_q_est_vs_q_true_diff(t_arr, q_des, vsrs.est.q); % control error

%% Part 3b: q-method

% Visors class
vsrs = Visors(w0, q0);

% Options
vsrs.opts.calc_q_det_flag = 0; % 1 if fictitious meas, 0 if normal meas
vsrs.opts.corrupt_measurements = add_noise; % 1 if introducing noise to meas
vsrs.opts.est_q = @vsrs.calc_q_stat; % deterministic or statistical?

% Propagate
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr);

% Plots
%plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q); %5
% plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q); %6 7.3.3
plot_q_est_vs_q_true_diff(t_arr, q_des, vsrs.est.q); % control error

%% Part 3c: q from angular velocity

%plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q_from_w); %7
% plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q_from_w); %8 7.3.4
plot_q_est_vs_q_true_diff(t_arr, q_des, vsrs.est.q_from_w); % control error

%% Part 7: attitude control error




