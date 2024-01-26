function plotRobotStates(q,param,t_vec,dimset,folder_name,sum_set)

arguments
    q
    param
    t_vec
    dimset
    folder_name
    sum_set = [1]
end
%PLOTROBOTSTATES この関数の概要をここに記述
%   詳細説明をここに記述
    %titleset = ["$\theta(t)$","$\dot{\theta}(t)$","$l(t)$","$\dot{l}(t)$","$X(t)$","$\dot{X}(t)$"];
    %ylabset = ["Angle (rad)","Anglular velocity (rad/s)","Length (m)","Length Velocity (m/s)","Position (m)","Velocity (m/s)"];
    ylabset = ["$\theta(t)$ (rad)","$\dot{\theta}(t)$ (rad/s)","$l(t)$ (m)","$\dot{l}(t)$ (m/s)","$X(t)$ (m)","$\dot{X}(t)$ (m/s)","$r(t),l(t)$ (m)","$\dot{r}(t),\dot{l}(t)$ (m/s)"];
    legset = ["$\theta(t)$","$\dot{\theta}(t)$","$l(t)$","$\dot{l}(t)$","$X(t)$","$\dot{X}(t)$","$r(t)$","$\dot{r}(t)$"];
    legset_2 = ["","","","","","","$l(t)$","$\dot{l}(t)$"];
    figure
    k = 0;
    for m = 1:size(dimset,1)
        for n = 1:size(dimset,2)
            k = k+1;
            subplot(size(dimset,1),size(dimset,2),k)
            if length(sum_set) == 1
                plot(t_vec, q(dimset(m,n),:,sum_set(1)));
                hold on
                if (ismember(dimset(m,n),[7,8]))    % also show l,l_dot when show r, r_dot
                    plot(t_vec, q(dimset(m,n)-4,:,sum_set(1)));
                    legend(legset(dimset(m,n)),legset_2(dimset(m,n)),'Interpreter','latex')
                else
                    legend(legset(dimset(m,n)),'Interpreter','latex')
                end
            else
                for s = sum_set
                    plot(t_vec, q(dimset(m,n),:,s),'Color','b','LineWidth',0.8);
                    hold on
                    if (ismember(dimset(m,n),[7,8]))    % also show l,l_dot when show r, r_dot
                        plot(t_vec, q(dimset(m,n)-4,:,s),'Color','r','LineWidth',0.8);
                    end
                end
            end
            hold off
            xlabel("Time (s)",'Interpreter','latex');
            ylabel(ylabset(dimset(m,n)),'Interpreter','latex');
            %title(titleset(dimset(m,n)),'Interpreter','latex');
            %legend(legset(),'Interpreter','latex')
        end
    end
    saveas(gcf,folder_name+'states.fig')
end

