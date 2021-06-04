% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;
clrs = DefaultColors();

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = dcm2quat(s.A_rot);

% Sim time parameters
t0 = 0; dt = 10; tf = 60*60; t_arr = (t0:dt:tf)'; N = numel(t_arr);
t_min = t_arr./60;

% flags: 
flags = [1,1,1,1,1,2,1];
    % 1: gravity gradient
    % 2: solar radiation pressure
    % 3: drag
    % 4: magnetic field
    % 5: EKF (1), q-method (2), or true state (0)
    % 6: linear (1), nonlinear (2), or no control (0)
    % 7: control noise

% Visors class
vsrs = Visors(w0, q0);
vsrs.opts.calc_q_det_flag = 0; % 1 if fictitious meas, 0 if normal meas
vsrs.opts.corrupt_measurements = 1; % 1 if introducing noise to meas
vsrs.opts.est_q = @vsrs.calc_q_stat; % deterministic or statistical?
[omega_out, quat_out, rv_ECI_out, M_out, EKF_out, states, L_dot] = vsrs.propagate(t_arr,flags);

plot_attitude_3D(vsrs.ICs, quat_out);

%% Plots
plt_rng = 10:(N-2);

%% actuation
figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
for i = 1:4
    subplot(4,1,i); hold on; grid on;
    plot(t_min,L_dot(i,:,1)-L_dot(i,:,2),'Color',clrs(1,:));
    xlabel('Time (min)'); ylabel('$\delta\dot{L} (Nm)$','interpreter','latex');
%     legend();
end

y_max = max(max(abs(M_out(:,:,6)-M_out(:,:,7))));
y_max = max(1E-9,y_max);
    
figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
for i = 1:3
    subplot(3,1,i); hold on; grid on;
    plot(t_min,M_out(i,:,6)-M_out(i,:,7),'Color',clrs(1,:),'DisplayName','M_{des}');
    xlabel('Time (min)'); ylabel('\delta M_c (Nm)');
%     legend();
%     ylim([-y_max y_max]);
end
% saveas(gcf,'figures/09-dM.png')

%% angular velocity
figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
    for i = 1:3
        subplot(3,1,i); hold on; grid on;
        plot(t_min(plt_rng),states.w_true(i,plt_rng)-states.w_est(i,plt_rng),'Color',clrs(1,:),'DisplayName','Attitude Determination Error');
        plot(t_min(plt_rng),states.w_des(i,plt_rng)-states.w_est(i,plt_rng),'Color',clrs(2,:),'LineStyle','-','DisplayName','Attitude Control Error');
        xlabel('Time (min)'); ylabel(['\delta \omega_',num2str(i)]);
        legend()
%         ylim([-y_max y_max]);
    end
% saveas(gcf,'figures/09-dw.png')
    
%% quaternions
figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
    for i = 1:4
        subplot(4,1,i); hold on; grid on;
        plot(t_min(plt_rng),states.q_true(i,plt_rng)-states.q_est(i,plt_rng),'Color',clrs(1,:),'DisplayName','Attitude Determination Error');
        plot(t_min(plt_rng),states.q_des(i,plt_rng)-states.q_est(i,plt_rng),'Color',clrs(2,:),'LineStyle','-','DisplayName','Attitude Control Error');
        xlabel('Time (min)'); ylabel(['\delta q_',num2str(i)]);
        legend()
%         ylim([-y_max y_max]);
    end
% save(gcf,'figures/09-dq.png')
%% EKF Error
plot_est_vs_true(t_min(plt_rng),states.w_true(:,plt_rng),EKF_out.mu_arr(1:3,plt_rng),EKF_out.cov_arr(1:3,1:3,plt_rng),'\omega_',[])
% saveas(gcf,'figures/09-w-ekf.png')
plot_est_vs_true(t_min(plt_rng),states.q_true(:,plt_rng),EKF_out.mu_arr(4:7,plt_rng),EKF_out.cov_arr(4:7,4:7,plt_rng),'q_',[])
% saveas(gcf,'figures/09-q-ekf.png')

%% Residuals
R_st = (deg2rad(40/3600))^2 * eye(3);
R_imu = (1.7453E-4)^2 * eye(3);
R = blkdiag(R_imu,R_st,R_st);
std_err = sqrt(diag(R));
plot_residuals(t_min(plt_rng),states.w_true(:,plt_rng)',EKF_out.z_post_arr(1:3,plt_rng)',std_err(1:3),'\omega_',' Residuals');
% saveas(gcf,'figures/09-w-res.png')
plot_residuals(t_min(plt_rng),EKF_out.z_pre_arr(4:6,plt_rng)',EKF_out.z_post_arr(4:6,plt_rng)',std_err(4:6),'m_{1,','} Residuals');
% saveas(gcf,'figures/09-m1-res.png')
plot_residuals(t_min(plt_rng),EKF_out.z_pre_arr(7:9,plt_rng)',EKF_out.z_post_arr(7:9,plt_rng)',std_err(7:9),'m_{2,','} Residuals');
% saveas(gcf,'figures/09-m2-res.png')
    