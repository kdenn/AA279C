function plot_w_est_vs_w_true_diff(t_arr, w_true, w_est, w_cov)
% Purpose: Plot difference between estimated quaternion components and true 
%          quaternion components (i.e. attitude estimation error)
%
% Inputs: t_arr  - array of time indices in seconds, size (1,N)
%         q_true - true attitude, size (1,N)
%         q_est  - estimated attitude, size (1,N-1)

% Seconds to minutes conversion
t_arr = t_arr ./ 60;
clrs = DefaultColors();
w_cov = sqrt(abs(w_cov));

% Resize so dimensions match
if size(w_est,2) == (size(w_true,2)-1)
    t_arr = t_arr(1:end-1);
    w_true = w_true(:,1:end-1);
end
N = numel(t_arr);

% Difference between the two
w_diff = w_est - w_true;

y_max = max(max(abs(w_diff)));

% Std deviation  calculations
temp = rad2deg(std(w_diff, 0, 2));
sqrt(sum(temp.^2));

figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
for i = 1:3
    y = 3.*reshape(w_cov(i,i,:),[N,1]);
    subplot(3,1,i); hold on; grid on;
%     plotFillRegion(t_arr,y,-y,clrs(1,:),0.7);
    plot(t_arr,w_diff(i,:),'Color',clrs(1,:));
    plot(t_arr,y,'Color',clrs(2,:));
    plot(t_arr,-y,'Color',clrs(2,:));
    xlabel('Time (min)'); ylabel(['\omega_',num2str(i),' error']);
    ylim([-y_max y_max]);
end

end
