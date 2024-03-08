function plotMultiBarError(x_list, val, color_base, legend_list, xlabel_name, val_name, folder_name)
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
method_order = [2 1 3 4];
x_cate = string(x_list);
x_cate = categorical(x_cate, x_cate);
b = bar(x_cate, permute(mean(val(:,method_order,:),1),[3,2,1]));
color_base = color_base(method_order);
for i = method_order
    hold on
    b(i).FaceColor = color_base(i);
    %plot(b(i).XEndPoints,b(i).YEndPoints,'Color',color_base(i),'Marker','o','MarkerFaceColor',color_base(i))
    %plot(b(i).XEndPoints,b(i).YEndPoints,'Color',color_base(i))
    errorbar(b(i).XEndPoints,b(i).YEndPoints,zeros(1,Nsim),permute(std(val(:,i,:),1),[3,1,2]),'LineStyle','none','Color',color_base(i))
    %b(i).Visible = 'off';
end

legend(legend_list(method_order),'Location','northwest')
xlabel(xlabel_name)
ylabel(val_name)
title(val_name)
%xlim([x_list(1) x_list(end)])
%grid on
saveas(gcf,folder_name+"/"+val_name+".fig")
saveas(gcf,folder_name+"/"+val_name+".png")
saveas(gcf,folder_name+"/"+val_name+".eps")
end

