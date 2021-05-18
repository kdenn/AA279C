function plot_q_est_vs_q_true_diff(t_arr, q_true, q_est, q_cov)
% Purpose: Plot difference between estimated quaternion components and true 
%          quaternion components (i.e. attitude estimation error)
%
% Inputs: t_arr  - array of time indices in seconds, size (1,N)
%         q_true - true attitude, size (1,N)
%         q_est  - estimated attitude, size (1,N-1)

% Seconds to minutes conversion
t_arr = t_arr ./ 60;
clrs = DefaultColors();
q_cov = sqrt(abs(q_cov));

% Resize so dimensions match
if size(q_est,2) == (size(q_est,2)-1)
    t_arr = t_arr(1:end-1);
    q_true = q_true(:,1:end-1);
end
N = numel(t_arr);

% Difference between the two
q_diff = q_est - q_true;

y_max = max(max(abs(q_diff)));

% Std deviation  calculations
temp = rad2deg(std(q_diff, 0, 2));
sqrt(sum(temp.^2));

figure(); 
for i = 1:4
    y = 3.*reshape(q_cov(i,i,:),[N,1]);
    subplot(4,1,i); hold on; grid on;
    plot(t_arr, q_diff(i,:),'Color',clrs(1,:));
    plot(t_arr,y,'Color',clrs(2,:));
    plot(t_arr,-y,'Color',clrs(2,:));
    xlabel('Time (min)'); ylabel('q_2 error');
    ylim([-y_max y_max]);
end
end
