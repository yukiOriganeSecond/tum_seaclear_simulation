function snapPath(t,q,x,param)
%SNAPPATH この関数の概要をここに記述
%   詳細説明をここに記述
plot(q(5,:), zeros(1,param.Nt),'--','LineWidth',0.7);
hold on
plot(x(1,:), x(3,:),'--','LineWidth',0.7);
line([x(1,t),q(5,t)],[x(3,t),0],'Color','k','LineWidth',0.7);
plot(x(1,t), x(3,t),'o','MarkerFaceColor','k','MarkerEdgeColor','none','MarkerSize',10);
plot(q(5,t),0,'+','Color','k','MarkerSize',10);
hold off
axis ij equal
xlabel("Position (m)")
ylabel("Depth (m)")
legend("vessel path","robot path","wire","robot position","crane top")
xlim([-3,3])
ylim([-1,7])
end

