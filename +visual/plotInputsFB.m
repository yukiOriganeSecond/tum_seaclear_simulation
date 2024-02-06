function plotInputsFB(u_nominal,u_fb,f,param,t_vec,dimset,folder_name,snum_set)
%PLOTINPUTSFB plot nominal control inputs u and with FB input u_val
    ylabset = ["$u_\theta(t)$ (N)","$u_r(t)$ (N)","$U_l(t)$ (N)","$U_X(t)$ (N)"];
    legset = ["$u^r_\theta(t)$ (N)","$u^r_r(t)$ (N)","$U^r_l(t)$","$U^r_X(t)$"];
    legset2 = ["$u_\theta(t)$ (N)","$u_r(t)$ (N)","$U_l(t)$","$U_X(t)$"];
    legset3 = ["$f_\theta(t)$ (N)","$f_r(t)$ (N)","$F_l(t)$","$F_X(t)$"];
    figure
    k = 0;
    for m = 1:size(dimset,1)
        for n = 1:size(dimset,2)
            k = k+1;
            subplot(size(dimset,1),size(dimset,2),k)
            plot(t_vec(1:end-1), u_nominal(dimset(m,n),1:end-1),'-','Color',"r",'LineWidth',1.0);
            hold on
            
            if length(snum_set) == 1
                plot(t_vec(1:end-1), u_fb(dimset(m,n),1:end-1),'-','Color',"#0072BD",'LineWidth',0.8);
                plot(t_vec(1:end-1), f(dimset(m,n),1:end-1,1));
                legend([legset(dimset(m,n)),legset2(dimset(m,n)),legset3(dimset(m,n))],'Interpreter','latex')
            else
                for s = snum_set
                   % plot(t_vec(1:end-1), f(dimset(m,n),1:end-1,s),'-','Color',"#D95319",'LineWidth',0.8);
                    plot(t_vec(1:end-1), u_fb(dimset(m,n),1:end-1,s),'-','Color',"#0072BD",'LineWidth',0.8);
                    plot(t_vec(1:end-1), u_nominal(dimset(m,n),1:end-1),'-','Color',"r",'LineWidth',1.2);
                end
            end
            hold off
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
    saveas(gcf,folder_name+'inputs.fig')
end

