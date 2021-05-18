function plot_w_est_vs_w_true_diff(t_arr, w_true, w_est)
% Purpose: Plot difference between estimated quaternion components and true 
%          quaternion components (i.e. attitude estimation error)
%
% Inputs: t_arr  - array of time indices in seconds, size (1,N)
%         q_true - true attitude, size (1,N)
%         q_est  - estimated attitude, size (1,N-1)

% Seconds to minutes conversion
t_arr = t_arr ./ 60;

% Resize so dimensions match
if size(w_est,2) == (size(w_true,2)-1)
    t_arr = t_arr(1:end-1);
    w_true = w_true(:,1:end-1);
end

% Difference between the two
w_diff = w_est - w_true;

y_max = max(max(abs(w_diff)));

% Std deviation  calculations
temp = rad2deg(std(w_diff, 0, 2));
sqrt(sum(temp.^2));

figure(); 
subplot(3,1,1); hold on; grid on;
plot(t_arr, w_diff(1,:), 'b');
xlabel('Time (min)'); ylabel('\omega_1 error');
ylim([-y_max y_max]);

subplot(3,1,2); hold on; grid on;
plot(t_arr, w_diff(2,:), 'b');
xlabel('Time (min)'); ylabel('\omega_2 error');
ylim([-y_max y_max]);

subplot(3,1,3); hold on; grid on;
plot(t_arr, w_diff(3,:), 'b');
xlabel('Time (min)'); ylabel('\omega_3 error');
ylim([-y_max y_max]);

end
