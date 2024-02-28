
clear
clc

scenario_name_detail = "path_20";

simulation_folder_name = "multiple_0228";
simulation_name_list = ["error_001", "error_002", "error_005"];
parameter_error_list = [0.01, 0.02, 0.05];
%simulation_name_list = ["error_010", "error_020", "error_050"];
%parameter_error_list = [0.10, 0.20, 0.50];
Nplan = 10;                                   % number of sample for planning
Nsim = 10;                                    % number of sample for simulation

kill_all_visualize = true;
method_container = MethodContainer;

method_container = method_container.addMethod("RA_SAA_alpha_001","RA-SAA",["alpha",0.01]);
method_container = method_container.addMethod("RA_SAA_alpha_002","RA-SAA",["alpha",0.02]);
method_container = method_container.addMethod("RA_SAA_alpha_005","RA-SAA",["alpha",0.05]);
method_container = method_container.addMethod("RA_SAA_alpha_010","RA-SAA",["alpha",0.10]);
method_container = method_container.addMethod("RA_SAA_alpha_020","RA-SAA",["alpha",0.20]);
method_container = method_container.addMethod("RA_SAA_alpha_050","RA-SAA",["alpha",0.50]);

method_container = method_container.addMethod("RA_SAA_PID_alpha_001","RA-SAA-PID",["alpha",0.01]);
method_container = method_container.addMethod("RA_SAA_PID_alpha_002","RA-SAA-PID",["alpha",0.02]);
method_container = method_container.addMethod("RA_SAA_PID_alpha_005","RA-SAA-PID",["alpha",0.05]);
method_container = method_container.addMethod("RA_SAA_PID_alpha_010","RA-SAA-PID",["alpha",0.10]);
method_container = method_container.addMethod("RA_SAA_PID_alpha_020","RA-SAA-PID",["alpha",0.20]);
method_container = method_container.addMethod("RA_SAA_PID_alpha_050","RA-SAA-PID",["alpha",0.50]);

%% run
if isfolder("data/multi_scenario/"+simulation_folder_name)
    fig = uifigure;
    selection = uiconfirm(fig, ...
    "folder "+simulation_folder_name+" already exists. Overrite folder?","Confirm Overrite", ...
    "Icon","warning","Options",["OK","Cancel"]);
    close all hidden
    if (selection == "Cancel")
        disp("process is Canceled")
        return
    end
end
mkdir("data/multi_scenario/"+simulation_folder_name)

sim_cnt = 0;
for simulation_name = simulation_name_list
    sim_cnt = sim_cnt + 1;
    simulation_name_full = simulation_folder_name+"/"+simulation_name;
    runMultiScenarioSimulation(method_container, scenario_name_detail,simulation_name_full,Nsim,Nplan,parameter_error_list(sim_cnt),kill_all_visualize);
end