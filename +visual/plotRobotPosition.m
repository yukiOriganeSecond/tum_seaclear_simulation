function plotRobotPosition(q,x,xd,param,t_vec)
%PLOTROBOTPOSITION この関数の概要をここに記述
%   詳細説明をここに記述
    % time response of robot's position
    figure
    plot(t_vec, x(1,:), t_vec, x(3,:));
    hold on
    yline(xd(1),'--','LineWidth',1,'Color',[0 0.4470 0.7410])
    yline(xd(3),'--','LineWidth',1,'Color',[0.8500 0.3250 0.0980])
    axis xy
    xlabel("time (s)")
    ylabel("position (m)")
    legend(["$x(t)$","$d(t)$","$x_d$","$d_d$"],'Interpreter','latex')
end

