function plotRobotOutputs(x,xd,param,t_vec,dimset,folder_name)
%PLOTROBOTSTATES この関数の概要をここに記述
%   詳細説明をここに記述
    %titleset = ["$\theta(t)$","$\dot{\theta}(t)$","$l(t)$","$\dot{l}(t)$","$X(t)$","$\dot{X}(t)$"];
    %ylabset = ["Position (m)","Velocity (m/s)","Depth (m)","Velocity (m/s)"];
    ylabset = ["$x(t)$ (m)","$\dot{x}(t)$ (m/s)","$d(t)$ (m)","$\dot{d}(t)$ (m/s)"];
    legset_1 = ["$x(t)$","$\dot{x}(t)$","$d(t)$","$\dot{d}(t)$"];
    legset_2 = ["$x_d$","$\dot{x}_d$","$d_d$","$\dot{d}_d$"];
    figure
    k = 0;
    for m = 1:size(dimset,1)
        for n = 1:size(dimset,2)
            k = k+1;
            subplot(size(dimset,1),size(dimset,2),k)
            plot(t_vec, x(dimset(m,n),:));
            hold on
            if ismember(dimset(m,n),[3,4])
                axis ij
            end
            yline(xd(dimset(m,n)),'--','LineWidth',1)
            xlabel("Time (s)",'Interpreter','latex');
            ylabel(ylabset(dimset(m,n)),'Interpreter','latex');
            %title(titleset(dimset(m,n)),'Interpreter','latex');
            legend([legset_1(dimset(m,n)),legset_2(dimset(m,n))],'Interpreter','latex')
        end
    end
    saveas(gcf,folder_name+'outputs.fig')
end

