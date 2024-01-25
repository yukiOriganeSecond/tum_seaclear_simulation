function plotInputs(u,param,t_vec,dimset,folder_name)
%PLOTINPUT plot control inputs u
    ylabset = ["$u_\theta(t)$ (N)","$u_r(t)$ (N)","$U_l(t)$ (N)","$U_X(t)$ (N)"];
    legset = ["$u_\theta(t)$ (N)","$u_r(t)$ (N)","$U_l(t)$","$U_X(t)$"];
    figure
    k = 0;
    for m = 1:size(dimset,1)
        for n = 1:size(dimset,2)
            k = k+1;
            subplot(size(dimset,1),size(dimset,2),k)
            plot(t_vec(1:end-1), u(dimset(m,n),1:end-1));
            xlim([0,t_vec(end)])
            hold on
            %yline(xd(1),'--','LineWidth',1,'Color')
            xlabel("Time (s)",'Interpreter','latex');
            ylabel(ylabset(dimset(m,n)),'Interpreter','latex');
            %title(titleset(dimset(m,n)),'Interpreter','latex');
            legend(legset(dimset(m,n)),'Interpreter','latex')
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

