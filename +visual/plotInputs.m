function plotInputs(u,f,param,t_vec,dimset,folder_name,snum_set,visualize_)
    arguments
        u
        f
        param
        t_vec
        dimset
        folder_name
        snum_set
        visualize_ = true;
    end
%PLOTINPUT plot control inputs u
    ylabset = ["$u_\theta(t)$ (N)","$u_r(t)$ (N)","$U_l(t)$ (N)","$U_X(t)$ (N)"];
    legset = ["$u_\theta(t)$ (N)","$u_r(t)$ (N)","$U_l(t)$","$U_X(t)$"];
    legset2 = ["$f_\theta(t)$ (N)","$f_r(t)$ (N)","$F_l(t)$","$F_X(t)$"];

    if visualize_
        h = figure('Visible','on');
    else
        h = figure('Visible','off');
    end

    k = 0;
    for m = 1:size(dimset,1)
        for n = 1:size(dimset,2)
            k = k+1;
            subplot(size(dimset,1),size(dimset,2),k)
            if length(snum_set) == 1
                plot(t_vec(1:end-1), u(dimset(m,n),1:end-1),'-','Color',"#0072BD",'LineWidth',0.8);
                hold on
                plot(t_vec(1:end-1), f(dimset(m,n),1:end-1,1));
                legend([legset(dimset(m,n)),legset2(dimset(m,n))],'Interpreter','latex')
            else
                for s = snum_set
                   plot(t_vec(1:end-1), f(dimset(m,n),1:end-1,s),'-','Color',"#0072BD",'LineWidth',0.8);
                   hold on
                   % plot(t_vec(1:end-1), u_fb(dimset(m,n),1:end-1,s),'-','Color',"#0072BD",'LineWidth',0.8);
                   % plot(t_vec(1:end-1), u_nominal(dimset(m,n),1:end-1),'-','Color',"r",'LineWidth',1.2);
                end
            end
            xlim([0,t_vec(end)])
            yl = ylim;
            if (yl(2)-yl(1))<2
                ylim([-1 1])
            end
            %yline(xd(1),'--','LineWidth',1,'Color')
            xlabel("Time (s)",'Interpreter','latex');
            ylabel(ylabset(dimset(m,n)),'Interpreter','latex');
            %title(titleset(dimset(m,n)),'Interpreter','latex');
        end
    end
    %figure
    %plot(t_vec, u(1,:), t_vec, u(2,:), t_vec, u(3,:));
    %axis xy
    %xlabel("time (s)")
    %ylabel("force input (N)")
    %legend(["$u$","$U_l$","$U_X$"],'Interpreter','latex')
    saveas(h,folder_name+'inputs.fig')

    if ~visualize_
        close(h);
    end
end

