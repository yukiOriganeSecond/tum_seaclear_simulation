function plotScenarioCondition(make_list, scenario, folder_name, layout)
    arguments
        make_list
        scenario
        folder_name = []
        layout = []
    end

    if ~isempty(layout)
        figure
    end
    cnt_s = 0;
    mkdir(folder_name+"/scenario_condition_pictures/")

    for s = make_list
        cnt_s = cnt_s + 1;
        if isempty(layout)
            figure
        else
            subplot(layout(1),layout(2),cnt_s);
        end
        
        yline(0,'LineWidth',0.8,'Color',"#4DBEEE")  % water surface
        hold on
        plot(scenario(s).y0(1,1),scenario(s).y0(3,1),'o','MarkerFaceColor',"#77AC30")   % robot initial position
        plot(scenario(s).yd(1,1),scenario(s).yd(3,1),'o','MarkerFaceColor',"#D95319")   % robot terget position
        plot(scenario(s).y0(5,1),0,'+','MarkerFaceColor',"#77AC30",'MarkerEdgeColor',"#77AC30")                         % vessel initial position   
        plot(scenario(s).yd(5,1),0,'+','MarkerFaceColor',"#D95319",'MarkerEdgeColor',"#D95319")                         % vessel target position
        for j = 1:size(scenario(s).obs_pos,2)
            objpos(j,:) = [(scenario(s).obs_pos(:,j)-scenario(s).obs_size(1,j)).', (2*scenario(s).obs_size(1,j))*[1 1]];
            rectangle('Position',objpos(j,:),'Curvature',[1 1]);
        end
        hold off
        axis ij equal
        xlabel("Position (m)")
        ylabel("Depth (m)")
       % legend("water surface","y(0)","yd","X(0)","Xd")
        title("scenario "+string(s)+", termination time = "+string(scenario(s).termination_time)+"s")
        xlim([-4,4])
        ylim([-1,7])
        if isempty(layout)
            legend("water surface","y(0)","yd","X(0)","Xd")
            saveas(gcf,folder_name+"/scenario_condition_pictures/"+"scenario_"+sprintf("%03d",s)+".fig")
            saveas(gcf,folder_name+"/scenario_condition_pictures/"+"scenario_"+sprintf("%03d",s)+".png")
        end
    end

    if ~isempty(layout)
        legend("water surface","y(0)","yd","X(0)","Xd")
        saveas(gcf,folder_name+"/scenario_condition_pictures/"+"scenario_"+sprintf("%03d",make_list(1))+"_"+sprintf("%03d",make_list(end))+".fig")
        %saveas(gcf,folder_name+"/scenario_condition_pictures/"+"scenario_"+sprintf("%03d",s)+".png")
    end
end

