% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;

% Visors class
s = visorsStruct();

% Initial Conditions
w0 = deg2rad([s.n;0;0]);
R_ECI2RTN = rotECItoRTN(s.Om,s.incl,s.w,s.e,s.r_ECI_0);
R_ECI2prin = s.A_rot*R1(pi)*R2(-pi/2)*R_ECI2RTN; % body is aligned with RTN, x=N,z=R
q0 = dcm2quat(R_ECI2prin);
vsrs = Visors(w0, q0);

% Sim time parameters
t0 = 0; dt = 1; tf = 60*30; t_arr = (t0:dt:tf)';
N = numel(t_arr);

% Options
vsrs.opts.calc_q_det_flag = 0; % 1 if fictitious meas, 0 if normal meas
vsrs.opts.corrupt_measurements = 1; % 1 if introducing noise to meas
vsrs.opts.est_q = @vsrs.calc_q_stat; % deterministic or statistical?

q_des = vsrs.calc_q_des(t_arr);

%% 2.a State Transition Matrix
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr,[1,1,1,1,0]);
mu = zeros(7,N);
mu(:,1) = [w0,q0];
for i = 1:(N-1)
    mu(:,i+1) = f(mu(:,i), M_out(:,i,5), dt);
end

figure()


%% Propagate
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr);

% Plots
%plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q); %5
% plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q); %6 7.3.3
plot_q_est_vs_q_true_diff(t_arr, q_des, vsrs.est.q); % control error

%% Part 3c: q from angular velocity

%plot_q_est_vs_q_true(t_arr, vsrs.true.q, vsrs.est.q_from_w); %7
% plot_q_est_vs_q_true_diff(t_arr, vsrs.true.q, vsrs.est.q_from_w); %8 7.3.4
plot_q_est_vs_q_true_diff(t_arr, q_des, vsrs.est.q_from_w); % control error






