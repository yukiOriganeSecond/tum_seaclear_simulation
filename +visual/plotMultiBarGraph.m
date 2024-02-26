
function plotMultiBarGraph(method_list, var, bar_name, folder_name)
    figure
    list_ = categorical(method_list);
    list_ = reordercats(list_,method_list);
    bar(list_,var)
    title(bar_name)
    saveas(gcf,folder_name+"/"+bar_name+".fig")
    saveas(gcf,folder_name+"/"+bar_name+".fig")
end