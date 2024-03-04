
clc
clear
visual.visualInit("thin");

%%
simulation_folder_name = "multiple_0301";
visialize_graphics = false;     % if false, make figures in silent way
color_base = ["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F","#0000FF","#00FF00","#FF0000","#FF00FF","#00FFFF"];

each_simulation_folder_list = dir("data/multi_scenario/"+simulation_folder_name);
each_simulation_folder_list = func.deleteInfeasibleFolder(each_simulation_folder_list);

avg_relative_energy_consumption_list = [];
avg_collision_rate_list = [];
avg_relative_termination_position_error = [];
avg_termination_position_error = [];
max_termination_position_error = [];
avg_energy_consumption_list = [];
max_relative_energy_consumption_list = [];
max_relative_termination_position_error = [];
avg_infeasible_ratio_list = [];

for sim_index = 1:length(each_simulation_folder_list)
    % summarize each simulation
    simulation_folder_each = "data/multi_scenario/"+simulation_folder_name+"/"+each_simulation_folder_list(sim_index).name;
    load(simulation_folder_each+"/results.mat", "method_container", "scenario")
    load(simulation_folder_each+"/results.mat", "energy_consumption", "final_target_error_pos", "minimum_collision_torelance", "face_infeasible_solution")
    method_legend_list = method_container.getNameForLegend();
    if size(avg_collision_rate_list,1) ~= length(method_legend_list)    % if not be initialized, do initialize
        avg_collision_rate_list = zeros(length(method_legend_list), length(each_simulation_folder_list));
        max_collision_rate_list = zeros(length(method_legend_list), length(each_simulation_folder_list));
        avg_relative_energy_consumption_list = zeros(length(method_legend_list), length(each_simulation_folder_list));
        max_relative_energy_consumption_list = zeros(length(method_legend_list), length(each_simulation_folder_list));
        avg_energy_consumption_list = zeros(length(method_legend_list), length(each_simulation_folder_list));
        avg_relative_termination_position_error = zeros(length(method_legend_list), length(each_simulation_folder_list));
        avg_termination_position_error = zeros(length(method_legend_list), length(each_simulation_folder_list));
        max_termination_position_error = zeros(length(method_legend_list), length(each_simulation_folder_list));
        max_relative_termination_position_error = zeros(length(method_legend_list), length(each_simulation_folder_list));
        avg_infeasible_ratio_list = zeros(length(method_legend_list), length(each_simulation_folder_list));
    end
    standard_method_index = 2;  % The number of standard method to calculate relative value;
    avg_collision_rate_list(:,sim_index) = mean(mean(minimum_collision_torelance<0,1),3).';
    max_collision_rate_list(:,sim_index) = max(mean(minimum_collision_torelance<0,1),[],3).';
    avg_relative_energy_consumption_list(:,sim_index) = mean(mean(energy_consumption,1)./mean(energy_consumption(:,standard_method_index,:),1),3).';
    max_relative_energy_consumption_list(:,sim_index) = max(max(energy_consumption,[],1)./max(energy_consumption(:,standard_method_index,:),[],1),[],3).';
    avg_energy_consumption_list(:,sim_index) = mean(mean(energy_consumption,1),3);
    avg_relative_termination_position_error(:,sim_index) = mean(mean(final_target_error_pos,1)./mean(final_target_error_pos(:,standard_method_index,:),1),3).';
    avg_termination_error(:,sim_index) = mean(mean(final_target_error_pos,1),3);
    max_termination_error(:,sim_index) = max(max(final_target_error_pos,[],1),[],3);
    max_relative_termination_position_error(:,sim_index) = max(max(final_target_error_pos,[],1)./max(final_target_error_pos(:,standard_method_index,:),[],1),[],3).';
    avg_infeasible_ratio_list(:,sim_index) = mean(mean(face_infeasible_solution,1),3).';
end

%% 
x_list = [70 120 180 250];
xlabel_name = "average of mass";
legend_list = method_container.getNameForLegend();
savefig_folder_name = "data/multi_scenario/"+simulation_folder_name+"/fig";
mkdir(savefig_folder_name)
index_list = 1:length(legend_list);
visual.plotMultiCombine(x_list, index_list, avg_collision_rate_list, max_collision_rate_list, color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);
visual.plotMultiCombine(x_list, index_list, avg_relative_energy_consumption_list, max_relative_energy_consumption_list, color_base, legend_list, xlabel_name, "relative energy comsunption", savefig_folder_name);
visual.plotMultiCombine(x_list, index_list, avg_energy_consumption_list, [], color_base, legend_list, xlabel_name, "energy comsunption", savefig_folder_name);
visual.plotMultiCombine(x_list, index_list, avg_relative_termination_position_error, max_relative_termination_position_error, color_base, legend_list, xlabel_name, "relative termination position error", savefig_folder_name);
visual.plotMultiCombine(x_list, index_list, avg_relative_termination_position_error, [], color_base, legend_list, xlabel_name, "relative termination position error", savefig_folder_name);
visual.plotMultiCombine(x_list, index_list, avg_termination_error, [], color_base, legend_list, xlabel_name, "average termination error", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, [], max_termination_error, color_base, legend_list, xlabel_name, "maximum termination error", savefig_folder_name);
visual.plotMultiCombine(x_list, index_list, avg_infeasible_ratio_list, [], color_base, legend_list, xlabel_name, "infeasible ratio", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, avg_collision_rate_list, [], color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);

% legend("\alpha="+string([0.01, 0.02, 0.05, 0.10, 0.20, 0.50]))