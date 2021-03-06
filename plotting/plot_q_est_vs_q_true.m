function plot_q_est_vs_q_true(t_arr, q_true, q_est)
% Purpose: Plot estimated quaternion components vs true quaternion
%          components (i.e. attitude estimation error)
%
% Inputs: t_arr  - array of time indices in seconds, size (1,N)
%         q_true - true attitude, size (1,N)
%         q_est  - estimated attitude, size (1,N-1)

% Seconds to minutes conversion
t_arr = t_arr ./ 60;

% Resize so dimensions match
t_arr = t_arr(1:end-1);
q_true = q_true(:,1:end-1);

figure(); 
subplot(4,1,1); hold on; grid on;
plot(t_arr, q_true(1,:), 'b');
plot(t_arr, q_est(1,:), 'r-.');
xlabel('Time (min)'); ylabel('q_1');

subplot(4,1,2); hold on; grid on;
plot(t_arr, q_true(2,:), 'b');
plot(t_arr, q_est(2,:), 'r-.');
xlabel('Time (min)'); ylabel('q_2');

subplot(4,1,3); hold on; grid on;
plot(t_arr, q_true(3,:), 'b');
plot(t_arr, q_est(3,:), 'r-.');
xlabel('Time (min)'); ylabel('q_3');

subplot(4,1,4); hold on; grid on;
plot(t_arr, q_true(4,:), 'b');
plot(t_arr, q_est(4,:), 'r-.');
xlabel('Time (min)'); ylabel('q_4');
legend({'True', 'Est'}, 'Location', 'NorthWest');
end
