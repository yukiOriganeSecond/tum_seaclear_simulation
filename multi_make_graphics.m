
clc
clear

%%
simulation_folder_name = "multiple_0307";
visialize_graphics = false;     % if false, make figures in silent way
color_base = ["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F","#0000FF","#00FF00","#FF0000","#FF00FF","#00FFFF"];

%%
each_simulation_folder_list = dir("data/multi_scenario/"+simulation_folder_name);
each_simulation_folder_list = func.deleteInfeasibleFolder(each_simulation_folder_list);

%feasible_folder_list =  each_simulation_folder_list(:).name;
for sim_index = 1:length(each_simulation_folder_list)
    % figure of each result
    simulation_folder_each = "data/multi_scenario/"+simulation_folder_name+"/"+each_simulation_folder_list(sim_index).name;
    load(simulation_folder_each+"/results.mat", "method_container", "scenario")
    each_result_file_list = func.deleteInfeasibleFolder(dir(simulation_folder_each+"/variables"));
    for result_index = 1:length(each_result_file_list)
        disp("processing... simulation: "+each_simulation_folder_list(sim_index).name+", result: "+each_result_file_list(result_index).name)
        load(simulation_folder_each+"/variables/"+each_result_file_list(result_index).name, "u","f","q","x","param_nominal","s","seed_simulate");
        t_vec = param_nominal.dt:param_nominal.dt:param_nominal.dt*param_nominal.Nt;
        name_without_extension = erase(each_result_file_list(result_index).name,".mat");
        visual.plotInputs(u,f,param_nominal,t_vec,[1,2;3,4],simulation_folder_each+"/inputs/"+name_without_extension+"_",1:length(seed_simulate),visialize_graphics)
        visual.makeSnapsWithPoints(q,x,param_nominal,scenario(s),t_vec,simulation_folder_each+"/paths/"+name_without_extension+"_",[1],1:length(seed_simulate),visialize_graphics);
    end
    % summarize each simulation
    load(simulation_folder_each+"/results.mat", "energy_consumption", "final_target_error_pos", "minimum_collision_torelance", "face_infeasible_solution")
    method_legend_list = method_container.getNameForLegend();
    visual.plotMultiAverageMax(energy_consumption,color_base,method_legend_list,"energy consumption",simulation_folder_each)
    visual.plotMultiAverageMax(final_target_error_pos,color_base,method_legend_list,"termination position error",simulation_folder_each,"max",0.3,"slack variable")
    visual.plotMultiAverageMax(minimum_collision_torelance,color_base,method_legend_list,"collision torelance",simulation_folder_each,"min",0,"collision")
    visual.plotMultiAverageMax(mean(minimum_collision_torelance<0,1),color_base,method_legend_list,"collision rate",simulation_folder_each)
    visual.plotMultiBarGraph(method_legend_list,mean(mean(face_infeasible_solution,1),3),"infeasible ratio",simulation_folder_each)
    close all
end

%% extract part of method
for sim_index = 1:length(each_simulation_folder_list)
    % summarize each simulation
    simulation_folder_each = "data/multi_scenario/"+simulation_folder_name+"/"+each_simulation_folder_list(sim_index).name;
    load(simulation_folder_each+"/results.mat", "method_container", "scenario")
    each_result_file_list = func.deleteInfeasibleFolder(dir(simulation_folder_each+"/variables"));
    load(simulation_folder_each+"/results.mat", "energy_consumption", "final_target_error_pos", "minimum_collision_torelance", "face_infeasible_solution")
    method_legend_list = method_container.getNameForLegend();
    method_index_list = 7:12;
    folder_name_ = simulation_folder_each + "/RA-SAA-PID";
    mkdir(folder_name_)
    visual.plotMultiAverageMax(energy_consumption(:,method_index_list,:),color_base,method_legend_list(method_index_list),"energy consumption",folder_name_)
    visual.plotMultiAverageMax(final_target_error_pos(:,method_index_list,:),color_base,method_legend_list(method_index_list),"termination position error",folder_name_,"max",0.3,"slack variable")
    visual.plotMultiAverageMax(minimum_collision_torelance(:,method_index_list,:),color_base,method_legend_list(method_index_list),"collision torelance",folder_name_,"min",0,"collision")
    visual.plotMultiAverageMax(mean(minimum_collision_torelance(:,method_index_list,:)<0,1),color_base,method_legend_list(method_index_list),"collision rate",folder_name_)
    visual.plotMultiBarGraph(method_legend_list(method_index_list),mean(mean(face_infeasible_solution(:,method_index_list,:),1),3),"infeasible ratio",folder_name_)
    close all
end

