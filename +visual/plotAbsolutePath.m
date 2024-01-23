function plotAbsolutePath(q,x,param,t_vec,folder_name)
%PLOTABSOLUTEPATH この関数の概要をここに記述
%   詳細説明をここに記述
% Path of the robot in the absolute coodinate
figure
plot(x(1,:), x(3,:));
hold on
plot(x(1,1), x(3,1),'o','MarkerFaceColor','g','MarkerEdgeColor','none');
plot(x(1,param.Nt), x(3,param.Nt),'o','MarkerFaceColor','r','MarkerEdgeColor','none');
plot(x(1,1),0,'+','Color','g');
plot(x(1,param.Nt),0,'+','Color','r');
axis ij equal
xlabel("x (s)")
ylabel("d (m)")
legend("path","start position","end position","crane top (start)","crane top (end)")
xlim([-3,3])
ylim([-1,6])
title("Absolute position of robot and vessel")
saveas(gcf,folder_name+'absolute_path.fig')
end

