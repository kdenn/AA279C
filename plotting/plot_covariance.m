function plot_covariance(t_arr, cov_arr)

cov_arr = abs(sqrt(cov_arr));

% Seconds to minutes conversion
t_arr = t_arr ./ 60;

% Resize so dimensions match
if size(cov,3) == (numel(t_arr)-1)
    t_arr = t_arr(1:end-1);
end

y_max = max(max(max(abs(cov_arr))));


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
