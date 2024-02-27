function snapPathWithPoints(t,q,x,param,scenario,snum_list)
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
if param.consider_collision == true
    for j = 1:size(param.obs_pos,2)
        objpos(j,:) = [(param.obs_pos(:,j)-param.obs_size(1,j)).', (2*param.obs_size(:,j))*[1 1]];
        rectangle('Position',objpos(j,:),'Curvature',[1 1]);
    end
end
yline(0,'LineWidth',0.8,'Color',"#4DBEEE")  % water surface
hold on
plot(scenario.y0(1,1),scenario.y0(3,1),'o','MarkerFaceColor',"#77AC30",'MarkerEdgeColor',"#77AC30")   % robot initial position
plot(scenario.yd(1,1),scenario.yd(3,1),'o','MarkerFaceColor',"#D95319",'MarkerEdgeColor',"#D95319")   % robot terget position
plot(scenario.y0(5,1),0,'+','MarkerFaceColor',"#77AC30",'MarkerEdgeColor',"#77AC30")                         % vessel initial position   
plot(scenario.yd(5,1),0,'+','MarkerFaceColor',"#D95319",'MarkerEdgeColor',"#D95319")  
hold off
axis ij equal
xlabel("Position (m)")
ylabel("Depth (m)")
%legend("vessel path","robot path","wire","robot position","crane top")
xlim([-4,4])
ylim([-1,7])
end

