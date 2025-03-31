function getDistanceErrorCurve(position_err,b)

    max_e = max(position_err);
    block_size = length(position_err);
    cycle_time = 0.097;
    x = (1:block_size)*cycle_time;
    size_ = 20;
    
    figure('Position', [100, 100, 800, 150]);
    hold on;
    plot(x, position_err,'g-');
    % plot([b b]*cycle_time,[0 max_e+1.8],'g--')
    xlim([0 block_size+1]*cycle_time);
    ylim([0 max_e+2]);
    ylabel('(mm)','FontName','Times New Roman','FontSize',size_)
    xlabel('time (s)','FontName','Times New Roman','FontSize',size_)
    title('trajectory error','FontName','Times New Roman','FontSize',size_)
    set(gca, 'FontSize', size_, 'FontName', 'Times New Roman');
end

