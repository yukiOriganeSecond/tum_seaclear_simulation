function plotRelativePath(q,x,param,t_vec,folder_name)
%PLOTRELATIVEPATH この関数の概要をここに記述
%   詳細説明をここに記述
% Path of the robot in the relative coodinate
figure
plot(x(1,:)-q(5,:), x(3,:));
hold on
plot(x(1,1)-q(5,1), x(3,1),'o','MarkerFaceColor','g','MarkerEdgeColor','none');
plot(x(1,param.Nt)-q(5,param.Nt), x(3,param.Nt),'o','MarkerFaceColor','r','MarkerEdgeColor','none');
plot(0,0,'+','Color','k');
axis ij equal
xlabel("x (s)")
ylabel("d (m)")
legend("path","start position","end position","crane top")
xlim([-5,5])
ylim([-1,6])
title("Rerative position of robot to vessel")
saveas(gcf,folder_name+'relative_path.fig')
end

