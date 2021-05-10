% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = [-0.24955978816328; 0.661604044827341; 0.661761547188027; 0.249141836437215];

% Sim time parameters
t0 = 0; dt = 1; tf = 60*20; t_arr = (t0:dt:tf)';

% Set options for class
opts = struct();
opts.calc_q_det_flag = 1;
opts.corrupt_measurements = 1;

% Visors class (not struct)
vsrs = Visors(w0, q0, opts);
vsrs.opts.est_q = @vsrs.calc_q_det; % deterministic or statistical

[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr);

% [m1_true, m2_true] = vsrs.get_ref_vecs_true(vsrs.ICs.JD_epoch);
% [m1_meas, m2_meas, m1_true, m2_true] = vsrs.get_ref_vecs_meas(vsrs.ICs.JD_epoch, quat_out(:,1));
% q_out = vsrs.calc_q_stat(m1_meas, m2_meas, m1_true, m2_true);
% q_out - q0

%%

make_plot(t_arr, vsrs.true.q, vsrs.est.q);

function make_plot(t_arr, q_true, q_est)

t_arr = t_arr ./ 60;

figure(); 
subplot(4,1,1); hold on; grid on;
plot(t_arr(1:end-1), q_true(1,1:end-1), 'b');
plot(t_arr(1:end-1), q_est(1,1:end), 'r:');
xlabel('Time (min)'); ylabel('q_1');

subplot(4,1,2); hold on; grid on;
plot(t_arr(1:end-1), q_true(2,1:end-1), 'b');
plot(t_arr(1:end-1), q_est(2,1:end), 'r:');
xlabel('Time (min)'); ylabel('q_2');

subplot(4,1,3); hold on; grid on;
plot(t_arr(1:end-1), q_true(3,1:end-1), 'b');
plot(t_arr(1:end-1), q_est(3,1:end), 'r:');
xlabel('Time (min)'); ylabel('q_3');

subplot(4,1,4); hold on; grid on;
plot(t_arr(1:end-1), q_true(4,1:end-1), 'b');
plot(t_arr(1:end-1), q_est(4,1:end), 'r:');
xlabel('Time (min)'); ylabel('q_4');
legend({'True', 'Est'}, 'Location', 'NorthWest');
end
