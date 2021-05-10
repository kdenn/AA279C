function plot_q_est_vs_q_true_diff(t_arr, q_true, q_est)
% Purpose: Plot difference between estimated quaternion components and true 
%          quaternion components (i.e. attitude estimation error)
%
% Inputs: t_arr  - array of time indices in seconds, size (1,N)
%         q_true - true attitude, size (1,N)
%         q_est  - estimated attitude, size (1,N-1)

% Seconds to minutes conversion
t_arr = t_arr ./ 60;

% Resize so dimensions match
t_arr = t_arr(1:end-1);
q_true = q_true(:,1:end-1);

% Difference between the two
q_diff = q_est - q_true;

y_max = max(max(abs(q_diff)));

% Std deviation  calculations
temp = rad2deg(std(q_diff, 0, 2));
sqrt(sum(temp.^2));

figure(); 
subplot(4,1,1); hold on; grid on;
plot(t_arr, q_diff(1,:), 'b');
xlabel('Time (min)'); ylabel('q_1');
ylim([-y_max y_max]);

subplot(4,1,2); hold on; grid on;
plot(t_arr, q_diff(2,:), 'b');
xlabel('Time (min)'); ylabel('q_2');
ylim([-y_max y_max]);

subplot(4,1,3); hold on; grid on;
plot(t_arr, q_diff(3,:), 'b');
xlabel('Time (min)'); ylabel('q_3');
ylim([-y_max y_max]);

subplot(4,1,4); hold on; grid on;
plot(t_arr, q_diff(4,:), 'b');
xlabel('Time (min)'); ylabel('q_4');
ylim([-y_max y_max]);
end
