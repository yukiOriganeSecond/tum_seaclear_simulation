function snapPath(t,q,x,param,snum_list)
%SNAPPATH この関数の概要をここに記述
%   詳細説明をここに記述
for s = snum_list
    plot(x(1,:,s), x(3,:,s),'LineWidth',0.7,'Color','b');
    hold on
    plot(q(5,:,s), zeros(1,param.Nt),'--','LineWidth',0.7,'Color','r');
end
line([x(1,t,1),q(5,t,1)],[x(3,t,1),0],'Color','k','LineWidth',0.7);
plot(x(1,t,1), x(3,t,1),'o','MarkerFaceColor','k','MarkerEdgeColor','none','MarkerSize',10);
plot(q(5,t,1),0,'+','Color','k','MarkerSize',10);
hold off
axis ij equal
xlabel("Position (m)")
ylabel("Depth (m)")
%legend("vessel path","robot path","wire","robot position","crane top")
xlim([-4,4])
ylim([-1,7])
end

