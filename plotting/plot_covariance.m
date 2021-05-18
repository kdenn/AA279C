function plot_covariance(x,cov_arr,lbl)

clrs = DefaultColors();
cov_arr = abs(sqrt(cov_arr));
N = size(cov_arr,3);
Nd = size(cov_arr,1);

% Resize so dimensions match
if N == (numel(x)-1)
    x = x(1:end-1);
end

y_max = max(max(max(cov_arr)));

figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
for i = 1:Nd
    y = 3.*reshape(cov_arr(i,i,:),[N,1]);
    subplot(Nd,1,i); hold on; grid on;
    plotFillRegion(x,y,-y,clrs(1,:),0.5)
    xlabel('Time (min)'); ylabel(['\Sigma_',num2str(i),' ',lbl]);
    ylim([-y_max y_max]);
    legend('3\sigma')
end

end
