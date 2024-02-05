function makeSnapsFB(q,q_nonFB,q_nominal,x,x_nonFB,x_nominal,param,t_vec,folder_name,timeset,snum_list)
%MAKESNAPS この関数の概要をここに記述
%   詳細説明をここに記述
figure
k = 0;
for m = 1:size(timeset,1)
    for n = 1:size(timeset,2)
        k = k+1;
        subplot(size(timeset,1),size(timeset,2),k);
        visual.snapPathFB(timeset(m,n),q,q_nonFB,q_nominal,x,x_nonFB,x_nominal,param,snum_list);
        title(string(t_vec(timeset(m,n)))+"(s)")
        if (timeset(m,n) == 1)
            title("0(s)")
        end
    end
end
saveas(gcf,folder_name+'snaps.fig')
end

