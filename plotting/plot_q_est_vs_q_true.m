function plot_q_est_vs_q_true(t_arr, q_true, q_est)
% Purpose: Plot estimated quaternion components vs true quaternion
%          components (i.e. attitude estimation error)
%
% Inputs: t_arr  - array of time indices in seconds, size (1,N)
%         q_true - true attitude, size (1,N)
%         q_est  - estimated attitude, size (1,N-1)

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
