% Kaitlin Dennison
% Joshua Geiser
clear all; close all; clc;
clrs = DefaultColors();

% Initial Conditions
s = visorsStruct();
w0 = deg2rad([0;0;0]);
q0 = dcm2quat(s.A_rot);

% Sim time parameters
t0 = 0; dt = 1; tf = 60*10; t_arr = (t0:dt:tf)'; N = numel(t_arr);
t_min = t_arr./60;

% Visors class
vsrs = Visors(w0, q0);
vsrs.opts.calc_q_det_flag = 0; % 1 if fictitious meas, 0 if normal meas
vsrs.opts.corrupt_measurements = 1; % 1 if introducing noise to meas
vsrs.opts.est_q = @vsrs.calc_q_stat; % deterministic or statistical?
[omega_out, quat_out, rv_ECI_out, M_out, EKF_out, omega_est, quat_est] = vsrs.propagate(t_arr,[0,1,1,1,2,1,1]);

% plot_attitude_3D(vsrs.ICs, quat_out);

%% Plots
plot_est_vs_true(t_min,M_out(:,:,6),M_out(:,:,7),[],'M_{c,','}'); % plot error in actuation
plot_est_vs_true(t_min,vsrs.true.w_des,omega_out,[],'\omega_',[])
plot_est_vs_true(t_min,vsrs.true.q_des,quat_out,[],'q_',[])

y_max = max(max(abs(M_out(:,:,7))));
y_max = max(1E-9,y_max);
    
figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
for i = 1:3
    subplot(3,1,i); hold on; grid on;
    plot(t_min,M_out(i,:,6),'Color',clrs(1,:),'DisplayName','M_{des}');
    plot(t_min,M_out(i,:,7),'Color',clrs(2,:),'DisplayName','M_{act}');
    xlabel('Time (min)'); ylabel('M_c (Nm)');
    legend();
    ylim([-y_max y_max]);
end