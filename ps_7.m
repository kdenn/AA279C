% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;
clrs = DefaultColors();

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
R_st = (deg2rad(40/3600))^2 * eye(3);
R_imu = (1.7453E-4)^2 * eye(3);
R = blkdiag(R_imu,R_st,R_st);
std_err = sqrt(diag(R));

%% 2.a State Transition Matrix
% test with no env torques
%{
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr,[0,0,0,0,0]);
mu = zeros(7,N);
mu(:,1) = [w0;q0];
for i = 1:(N-1)
    mu(:,i+1) = f(mu(:,i), zeros(3,1), dt);
end

plot_w_est_vs_w_true_diff(t_arr, omega_out, mu(1:3,:))
plot_q_est_vs_q_true_diff(t_arr, quat_out, mu(4:end,:))
%}

%% Get w and q noise caused by each perturbation
%{
[omega_out_base, quat_out_base, ~, ~, ~] = vsrs.propagate(t_arr,[0,0,0,0,0]);

[omega_out_g, quat_out_g, ~, ~, ~] = vsrs.propagate(t_arr,[1,0,0,0,0]);
dw = abs(omega_out_base-omega_out_g);
dq = abs(quat_out_base-quat_out_g);
noises.Qw_g = diag(std(dw,0,2).^2);
noises.Qq_g = diag(std(dq,0,2).^2);
[omega_out_s, quat_out_s, ~, ~, ~] = vsrs.propagate(t_arr,[0,1,0,0,0]);
dw = abs(omega_out_base-omega_out_s);
dq = abs(quat_out_base-quat_out_s);
noises.Qw_s = diag(std(dw,0,2).^2);
noises.Qq_s = diag(std(dq,0,2).^2);
[omega_out_d, quat_out_d, ~, ~, ~] = vsrs.propagate(t_arr,[0,0,1,0,0]);
dw = abs(omega_out_base-omega_out_d);
dq = abs(quat_out_base-quat_out_d);
noises.Qw_d = diag(std(dw,0,2).^2);
noises.Qq_d = diag(std(dq,0,2).^2);
[omega_out_m, quat_out_m, ~, ~, ~] = vsrs.propagate(t_arr,[0,0,0,1,0]);
dw = abs(omega_out_base-omega_out_m);
dq = abs(quat_out_base-quat_out_m);
noises.Qw_m = diag(std(dw,0,2).^2);
noises.Qq_m = diag(std(dq,0,2).^2);
save('propagation/noises.mat','noises');
%}
 
%% 3 Test the Full EKF
[omega_out, quat_out, rv_ECI_out, M_out, EKF_out] = vsrs.propagate(t_arr,[1,0,0,0,1]);

%% Plots
plot_est_vs_true(t_arr./60,omega_out,EKF_out.mu_arr(1:3,:),EKF_out.cov_arr(1:3,1:3,:),'\omega_',[])
plot_est_vs_true(t_arr./60,quat_out,EKF_out.mu_arr(4:7,:),EKF_out.cov_arr(4:7,4:7,:),'q_',[])

plot_residuals(t_arr./60,EKF_out.z_pre_arr(1:3,:)',EKF_out.z_post_arr(1:3,:)',std_err(1:3),'\omega_',' Residuals');
plot_residuals(t_arr./60,EKF_out.z_pre_arr(4:6,:)',EKF_out.z_post_arr(4:6,:)',std_err(4:6),'m_{1,','} Residuals');
plot_residuals(t_arr./60,EKF_out.z_pre_arr(7:9,:)',EKF_out.z_post_arr(7:9,:)',std_err(7:9),'m_{2,','} Residuals');






