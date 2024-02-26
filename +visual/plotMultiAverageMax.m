function plotMultiAverageMax(val, color_base, method_list, val_name, folder_name, max_or_min, base_y, base_name)
%PLOTMULTIAVERAGEMAX この関数の概要をここに記述
%   詳細説明をここに記述
arguments
    val
    color_base
    method_list
    val_name
    folder_name
    max_or_min = "max";
    base_y = []
    base_name = []
end

Nm = size(val,2);
Nsc = size(val,3);
color_list = color_base(1:Nm);
figure
hold on
for j = 1:Nm
    if (size(val,1) == 1)
        plot(1:Nsc, permute(val(:,j,:),[2,3,1]),"-o",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
    elseif (size(val,1) > 2)
        plot(1:Nsc, permute(mean(val(:,j,:),1),[2,3,1]),"-o",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
        if max_or_min == "min"
            plot(1:Nsc, permute(min(val(:,j,:),[],1),[2,3,1]),"-+",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
        else
            plot(1:Nsc, permute(max(val(:,j,:),[],1),[2,3,1]),"-+",'Color',color_list(j),'MarkerEdgeColor',color_list(j),'MarkerFaceColor',color_list(j));
        end
    end
end
if (size(val,1) == 1)
    leg = method_list.';
elseif (size(val,1) > 2)
    leg = repelem(method_list.',2,1)+string(repmat([": average";": "+max_or_min],Nm,1));
end

if ~isempty(base_y)
    yline(base_y,'--b',LineWidth=0.8)
    leg = [leg; base_name];
end

legend(leg)
xlabel("scenario")
ylabel(val_name)
title(val_name)
saveas(gcf,folder_name+"/"+val_name+".fig")

end

