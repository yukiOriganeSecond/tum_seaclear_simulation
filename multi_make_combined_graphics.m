
clc
clear
visual.visualInit("thin");

%%
simulation_folder_name = "multiple_0307_from_desk";
visialize_graphics = false;     % if false, make figures in silent way
color_base = ["#EDB120","#D95319","#0072BD","#7E2F8E","#77AC30","#4DBEEE","#A2142F","#0000FF","#00FF00","#FF0000","#FF00FF","#00FFFF"];

each_simulation_folder_list = dir("data/multi_scenario/"+simulation_folder_name);
each_simulation_folder_list = func.deleteInfeasibleFolder(each_simulation_folder_list);

energy_consumption_list = [];
collision_rate_list = [];
termination_position_error = [];
infeasible_ratio_list = [];

Nsim = length(each_simulation_folder_list); % number of simulation setting (distribution)
for sim_index = 1:Nsim
    % summarize each simulation
    simulation_folder_each = "data/multi_scenario/"+simulation_folder_name+"/"+each_simulation_folder_list(sim_index).name;
    load(simulation_folder_each+"/results.mat", "method_container", "scenario")
    load(simulation_folder_each+"/results.mat", "energy_consumption", "final_target_error_pos", "minimum_collision_torelance", "face_infeasible_solution")
    method_legend_list = method_container.getNameForLegend();
    Nmethod = method_container.getNumberOfMethods();
    Nenv = length(scenario);
    if size(energy_consumption_list,2) ~= Nmethod    % if not be initialized, do initialize
        energy_consumption_list = zeros(Nenv,Nmethod,Nsim);
        collision_rate_list = zeros(Nenv,Nmethod,Nsim);
        termination_position_error = zeros(Nenv,Nmethod,Nsim);
        infeasible_ratio_list = zeros(Nenv,Nmethod,Nsim);
    end
    standard_method_index = 2;  % The number of standard method to calculate relative value;
    energy_consumption_list(:,:,sim_index) = permute(mean(energy_consumption,1),[3,2,1]);
    collision_rate_list(:,:,sim_index) = permute(mean(minimum_collision_torelance<0,1),[3,2,1]);
    termination_position_error(:,:,sim_index) = permute(mean(final_target_error_pos,1),[3,2,1]);
    infeasible_ratio_list(:,:,sim_index) = permute(mean(face_infeasible_solution,1),[3,2,1]);
end

%% 

%x_list = [0.01 0.02 0.05 0.10 0.20 0.50];
%xlabel_name = "error of model parameter";
x_list = [0.00 0.01 0.02];
xlabel_name = "error of model parameter";
%x_list = [70, 120, 180, 250];
%xlabel_name = "mass of robot";
legend_list = method_container.getNameForLegend();
savefig_folder_name = "data/multi_scenario/"+simulation_folder_name+"/fig/barerror/";
mkdir(savefig_folder_name)
visual.plotMultiBarError(x_list, collision_rate_list, color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);
visual.plotMultiBarError(x_list, infeasible_ratio_list, color_base, legend_list, xlabel_name, "infeasible rate", savefig_folder_name);
visual.plotMultiBarError(x_list, energy_consumption_list, color_base, legend_list, xlabel_name, "energy consumption", savefig_folder_name);
ylim([0 14000])
visual.plotMultiBarError(x_list, termination_position_error, color_base, legend_list, xlabel_name, "termination error", savefig_folder_name);

savefig_folder_name = "data/multi_scenario/"+simulation_folder_name+"/fig/ploterror/";
mkdir(savefig_folder_name)
%visual.plotMultiCombineError(x_list, collision_rate_list, color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);
%visual.plotMultiCombineError(x_list, infeasible_ratio_list, color_base, legend_list, xlabel_name, "infeasible rate", savefig_folder_name);
%visual.plotMultiCombineError(x_list, energy_consumption_list, color_base, legend_list, xlabel_name, "energy consumption", savefig_folder_name);
%visual.plotMultiCombineError(x_list, termination_position_error, color_base, legend_list, xlabel_name, "termination error", savefig_folder_name);

% savefig_folder_name = "data/multi_scenario/"+simulation_folder_name+"/fig/barchart/";
% mkdir(savefig_folder_name)
% visual.plotMultiBarchart(x_list, collision_rate_list, color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);
% visual.plotMultiBarchart(x_list, infeasible_ratio_list, color_base, legend_list, xlabel_name, "infeasible rate", savefig_folder_name);
% visual.plotMultiBarchart(x_list, energy_consumption_list, color_base, legend_list, xlabel_name, "energy consumption", savefig_folder_name);
% visual.plotMultiBarchart(x_list, termination_position_error, color_base, legend_list, xlabel_name, "termination error", savefig_folder_name);

savefig_folder_name = "data/multi_scenario/"+simulation_folder_name+"/fig/";
mkdir(savefig_folder_name)
%visual.plotMultiCombine(x_list, collision_rate_list, color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, avg_relative_energy_consumption_list, max_relative_energy_consumption_list, color_base, legend_list, xlabel_name, "relative energy comsunption", savefig_folder_name);
%visual.plotMultiCombine(x_list, energy_consumption_list, color_base, legend_list, xlabel_name, "energy comsunption", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, avg_relative_termination_position_error, max_relative_termination_position_error, color_base, legend_list, xlabel_name, "relative termination position error", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, avg_relative_termination_position_error, [], color_base, legend_list, xlabel_name, "relative termination position error", savefig_folder_name);
%visual.plotMultiCombine(x_list, termination_position_error, color_base, legend_list, xlabel_name, "termination position error", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, [], max_termination_error, color_base, legend_list, xlabel_name, "maximum termination error", savefig_folder_name);
%visual.plotMultiCombine(x_list, infeasible_ratio_list, color_base, legend_list, xlabel_name, "infeasible ratio", savefig_folder_name);
%visual.plotMultiCombine(x_list, index_list, avg_collision_rate_list, [], color_base, legend_list, xlabel_name, "collision rate", savefig_folder_name);

% legend("\alpha="+string([0.01, 0.02, 0.05, 0.10, 0.20, 0.50]))

%% ttest
clear tb_result
savefig_folder_name = "data/multi_scenario/"+simulation_folder_name+"/fig/";
mkdir(savefig_folder_name)
%x_list = [0.01 0.02 0.05 0.10 0.20 0.50];
x_list = [0.01 0.02 0.05];
tb_collision = func.calcTtest2(collision_rate_list,x_list);
tb_termination = func.calcTtest2(termination_position_error,x_list);
tb_infeasible = func.calcTtest2(infeasible_ratio_list,x_list);
tb_energy = func.calcTtest2(energy_consumption_list,x_list);
save(savefig_folder_name+"ttest.mat","tb_energy","tb_infeasible","tb_termination","tb_collision");
