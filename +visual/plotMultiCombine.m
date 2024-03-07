function plotMultiCombine(x_list, val, color_base, legend_list, xlabel_name, val_name, folder_name, base_y)
%PLOTMULTIAVERAGEMAX この関数の概要をここに記述
%   詳細説明をここに記述
arguments
    x_list
    val
    color_base
    legend_list
    xlabel_name
    val_name
    folder_name
    base_y = []
end


Nm = size(val,2);
color_list = color_base(1:Nm);
figure

for j = 1:Nm
   % if ~isempty(val)
        semilogx(x_list, permute(mean(val(:,j,:),1),[3,1,2]),"-o",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
        hold on
   % end
    
   % if ~isempty(val_max)
   %     plot(x_list, val_max(j,:),"-+",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
   %     hold on
   % end
end

%if ~isempty(val_max) && ~isempty(val)
%    leg = repelem(legend_list(index_list).',2,1)+string(repmat([": average";": max"],Nm,1));
%else
    leg = legend_list;
%end

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

