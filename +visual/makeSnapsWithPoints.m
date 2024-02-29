function makeSnapsWithPoints(q,x,param,scenario,t_vec,folder_name,timeset,snum_list,visualize_)
%MAKESNAPS この関数の概要をここに記述
%   詳細説明をここに記述
arguments
    q
    x
    param
    scenario
    t_vec
    folder_name
    timeset
    snum_list
    visualize_ = true
end
if visualize_
    h = figure('Visible','on');
else
    h = figure('Visible','off');
end
k = 0;
for m = 1:size(timeset,1)
    for n = 1:size(timeset,2)
        k = k+1;
        subplot(size(timeset,1),size(timeset,2),k);
        visual.snapPathWithPoints(timeset(m,n),q,x,param,scenario,snum_list);
        title(string(t_vec(timeset(m,n)))+"(s)")
        if (timeset(m,n) == 1)
            title("0(s)")
        end
    end
end
saveas(h,folder_name+'snaps.fig')
saveas(h,folder_name+'snaps.png')
if ~visualize_
    close(h);
end

end

