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
R_st = (deg2rad(40/3600))^2 * eye(3);
R_imu = (1.7453E-4)^2 * eye(3);
R = blkdiag(R_imu,R_st,R_st);
std_err = sqrt(diag(R));

%% 2.a State Transition Matrix
% test with no env torques
[omega_out, quat_out, rv_ECI_out, M_out] = vsrs.propagate(t_arr,[0,0,0,0,0]);
mu = zeros(7,N);
mu(:,1) = [w0;q0];
for i = 1:(N-1)
    mu(:,i+1) = f(mu(:,i), zeros(3,1), dt);
end

plot_w_est_vs_w_true_diff(t_arr, omega_out, mu(1:3,:))
plot_q_est_vs_q_true_diff(t_arr, quat_out, mu(4:end,:))

%% 3 Test the Full EKF
[omega_out, quat_out, rv_ECI_out, M_out, EKF_out] = vsrs.propagate(t_arr);

plot_w_est_vs_w_true_diff(t_arr, omega_out, EKF_out.mu_arr(1:3,:))
plot_q_est_vs_q_true_diff(t_arr, quat_out, EKF_out.mu_arr(4:end,:))

% measurement residuals
y_max = max(max(abs(w_diff)));

% w measurement residuals
figure(); 
    subplot(3,1,1); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(1,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(1,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(1),std_err(1)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

    subplot(3,1,2); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(2,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(2,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(2),std_err(2)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

    subplot(3,1,3); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(3,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(3,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(3),std_err(3)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

% m1 measurement residuals
figure(); 
    subplot(3,1,1); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(3+1,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(3+1,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(3+1),std_err(3+1)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

    subplot(3,1,2); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(3+2,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(3+2,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(3+2),std_err(3+2)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

    subplot(3,1,3); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(3+3,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(3+3,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(3+3),std_err(3+3)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

% m2 measurement residuals
figure(); 
    subplot(3,1,1); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(6+1,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(6+1,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(6+1),std_err(6+1)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

    subplot(3,1,2); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(6+2,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(6+2,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(6+2),std_err(6+2)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);

    subplot(3,1,3); hold on; grid on;
    plot(t_arr, EKF_out.z_pre_out(6+3,:),'b','DisplayName','Pre');
    plot(t_arr, EKF_out.z_post_out(6+3,:),'g','DisplayName','Post');
    plot(t_arr([1,end]),[std_err(6+3),std_err(6+3)],'r','STD Error');
    xlabel('Time (min)'); ylabel('Measurement Residuals');
    ylim([-y_max y_max]);






