function plotMultiCombine(x_list, index_list, val_avg, val_max, color_base, legend_list, xlabel_name, val_name, folder_name, base_y)
%PLOTMULTIAVERAGEMAX この関数の概要をここに記述
%   詳細説明をここに記述
arguments
    x_list
    index_list
    val_avg
    val_max
    color_base
    legend_list
    xlabel_name
    val_name
    folder_name
    base_y = []
end

if ~isempty(val_avg)
    val_avg = val_avg(index_list,:);
end
if ~isempty(val_max)
    val_max = val_max(index_list,:);
end
Nm = max(size(val_avg,1),size(val_max,1));
color_list = color_base(1:Nm);
figure

for j = 1:Nm
    if ~isempty(val_avg)
        semilogx(x_list, val_avg(j,:),"-o",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
        hold on
    end
    
    if ~isempty(val_max)
        semilogx(x_list, val_max(j,:),"-+",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
        hold on
    end
end

if ~isempty(val_max) && ~isempty(val_avg)
    leg = repelem(legend_list(index_list).',2,1)+string(repmat([": average";": max"],Nm,1));
else
    leg = legend_list(index_list);
end

if ~isempty(base_y)
    yline(base_y,'--b',LineWidth=0.8)
    leg = [leg; base_name];
end

legend(leg)
xlabel(xlabel_name)
ylabel(val_name)
title(val_name)
xlim([x_list(1) x_list(end)])
grid on
saveas(gcf,folder_name+"/"+val_name+".fig")
saveas(gcf,folder_name+"/"+val_name+".png")
end

