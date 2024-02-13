function snapPathFB(t,q,q_nonFB,~,x,~,x_nominal,param,snum_list)
%SNAPPATH この関数の概要をここに記述
%   詳細説明をここに記述
for s = snum_list
    plot(x(1,:,s), x(3,:,s),'LineWidth',0.7,'Color','b');
    hold on
    %plot(x_nonFB(1,:,s), x_nonFB(3,:,s),'LineWidth',0.7,'Color','g');
    plot(q(5,:,s), zeros(1,param.Nt),'--','LineWidth',0.7,'Color','r');
    plot(q_nonFB(5,:,s), zeros(1,param.Nt),'--','LineWidth',0.7,'Color','m');
end
 plot(x_nominal(1,:,s), x_nominal(3,:,s),'LineWidth',1.1,'Color','r');
line([x(1,t,1),q(5,t,1)],[x(3,t,1),0],'Color','k','LineWidth',0.7);
plot(x(1,t,1), x(3,t,1),'o','MarkerFaceColor','k','MarkerEdgeColor','none','MarkerSize',10);
plot(q(5,t,1),0,'+','Color','k','MarkerSize',10);
if param.consider_collision == true
    objpos = [(param.obs_pos-param.obs_size).', (2*param.obs_size)*[1 1]];
    rectangle('Position',objpos,'Curvature',[1 1]);
end
hold off
axis ij equal
xlabel("Position (m)")
ylabel("Depth (m)")
%legend("vessel path","robot path","wire","robot position","crane top")
xlim([-4,4])
ylim([-1,7])
end

