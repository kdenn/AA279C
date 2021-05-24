function plot_residuals(x,z_pre,z_post,noise_thr,lbl_pre,lbl_post)

clrs = DefaultColors();
y_max = max([max(max(z_pre(10:end,:))),max(max(z_post(10:end,:)))]);

figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
for i = 1:numel(noise_thr)
    subplot(numel(noise_thr),1,i); hold on; grid on;
    plot(x,z_pre(:,i),'Color',clrs(1,:))
    plot(x,z_post(:,i),'Color',clrs(2,:))
    plot(x([1,end]),[noise_thr(i),noise_thr(i)],'Color',clrs(3,:));
    plot(x([1,end]),-[noise_thr(i),noise_thr(i)],'Color',clrs(3,:));
    xlabel('Time (min)'); ylabel([lbl_pre,num2str(i),lbl_post]);
    ylim([-y_max y_max]);
    legend('Pre','Post','1\sigma Meas')
end