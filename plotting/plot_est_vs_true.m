function plot_est_vs_true(x,y_true,y_est,y_cov,lbl_pre,lbl_post)

    clrs = DefaultColors();
    y_cov = sqrt(abs(y_cov));

    % Resize so dimensions match
    if size(y_est,2) == (size(y_true,2)-1)
        x = x(1:end-1);
        y_true = y_true(:,1:end-1);
    end

    N = numel(x);
    Nd = size(y_true,1);
    y_diff = y_est - y_true;
    y_max = max(max(abs(y_diff(:,10:end))));

    figure(); set(gcf, 'Position',  [100, 100, 1200, 700]);
    for i = 1:Nd
        y = 3.*reshape(y_cov(i,i,:),[N,1]);
        subplot(Nd,1,i); hold on; grid on;
        plot(x,y_diff(i,:),'Color',clrs(1,:));
        plot(x,y,'Color',clrs(2,:));
        plot(x,-y,'Color',clrs(2,:));
        xlabel('Time (min)'); ylabel([lbl_pre,num2str(i),lbl_post]);
        ylim([-y_max y_max]);
        legend(['\delta',lbl_pre,num2str(i)],'3\sigma')
    end

end