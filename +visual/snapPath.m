function snapPath(t,q,x,param,snum_list)
%SNAPPATH この関数の概要をここに記述
%   詳細説明をここに記述
sim_index = 6;
for s = snum_list
    plot(x(1,:,s), x(3,:,s),'LineWidth',0.7,'Color','b');
    hold on
    plot(q(5,:,s), zeros(1,param.Nt),'--','LineWidth',0.7,'Color','r');
end
line([x(1,t,sim_index),q(5,t,sim_index)],[x(3,t,sim_index),0],'Color','k','LineWidth',0.7);
plot(x(1,t,sim_index), x(3,t,sim_index),'o','MarkerFaceColor','k','MarkerEdgeColor','none','MarkerSize',10);
plot(q(5,t,sim_index),0,'+','Color','k','MarkerSize',10);
if param.consider_collision == true
    for j = 1:size(param.obs_pos,2)
        objpos(j,:) = [(param.obs_pos(:,j)-param.obs_size(1,j)).', (2*param.obs_size(:,j))*[1 1]];
        rectangle('Position',objpos(j,:),'Curvature',[1 1]);
    end
end
hold off
axis ij equal
xlabel("Position (m)")
ylabel("Depth (m)")
%legend("vessel path","robot path","wire","robot position","crane top")
xlim([-4,4])
ylim([-1,7])
end

