function plotRobotVelocity(q,x,xd,param,t_vec)
%PLOTROBOTVELOCITY この関数の概要をここに記述
%   詳細説明をここに記述
% time response of robot's speed
figure
plot(t_vec, x(2,:), t_vec, x(4,:));
hold on
yline(xd(2),'--','LineWidth',1,'Color',[0 0.4470 0.7410])
yline(xd(4),'--','LineWidth',1,'Color',[0.8500 0.3250 0.0980])
axis xy
xlabel("time (s)")
ylabel("velocity (m/s)")
legend(["$\dot{x}(t)$","$\dot{x}(t)$","$\dot{x}_d$","$\dot{d}_d$"],'Interpreter','latex')
end

