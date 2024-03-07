function plotMultiBarchart(x_list, val, color_base, legend_list, xlabel_name, val_name, folder_name)
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
end

Nenv = size(val, 1);
Nmethod = size(val, 2);
Nsim = size(val, 3);

figure
x_mat = repmat(permute(x_list,[3,1,2]),Nenv,Nmethod);
leg_mat = repmat(legend_list,Nenv,1,Nsim);
tb = table(val(:),x_mat(:),leg_mat(:));
tb.Var2 = categorical(tb.Var2, x_list);
tb.Var3 = categorical(tb.Var3, legend_list);
b = boxchart(tb.Var2,tb.Var1,'GroupByColor',tb.Var3,'JitterOutliers','on','MarkerStyle','.');
%b.JitterOutliers = 'on';
%b.MarkerStyle = '.';

legend%(legend_list)
xlabel(xlabel_name)
ylabel(val_name)
title(val_name)
%xlim([x_list(1) x_list(end)])
%grid on
saveas(gcf,folder_name+"/"+val_name+".fig")
saveas(gcf,folder_name+"/"+val_name+".png")
end

