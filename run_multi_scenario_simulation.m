
clear
clc

simulation_folder_name = "multiple_0314_seacat";
scenario_name_detail = "path_100_modify";

% for PC 1
simulation_name_list = ["error_0.01", "error_0.02","error_0.05","error_0.10", "error_0.20","error_0.50"];
parameter_error_list = [0.01, 0.02, 0.05, 0.10, 0.20, 0.50];
robot_mass_list = [120, 120, 120, 120, 120, 120];

% for PC 2
% simulation_name_list = ["error_0.10"];
% parameter_error_list = [0.10];
% robot_mass_list = [120];

% for PC 3
% simulation_name_list = ["error_0.20","error_0.50"];
% parameter_error_list = [0.20, 0.50];
% robot_mass_list = [120, 120];

% for PC 4
% simulation_name_list = ["error_0.02"];
% parameter_error_list = [0.02];
% robot_mass_list = [120];

Nplan = 10;                                   % number of sample for planning
Nsim = 20;                                    % number of sample for simulation

kill_all_visualize = true;
method_container = MethodContainer;

%method_container = method_container.addMethod("RA_SAA_alpha_001","RA-SAA",["alpha",0.01]);
%method_container = method_container.addMethod("RA_SAA_alpha_002","RA-SAA",["alpha",0.02]);
%method_container = method_container.addMethod("RA_SAA_alpha_005","RA-SAA",["alpha",0.05]);
%method_container = method_container.addMethod("RA_SAA_alpha_010","RA-SAA",["alpha",0.10]);
%method_container = method_container.addMethod("RA_SAA_alpha_020","RA-SAA",["alpha",0.20]);
%method_container = method_container.addMethod("RA_SAA_alpha_050","RA-SAA",["alpha",0.50]);

%method_container = method_container.addMethod("RA_SAA_PID_alpha_001","RA-SAA-PID",["alpha",0.01]);
%method_container = method_container.addMethod("RA_SAA_PID_alpha_002","RA-SAA-PID",["alpha",0.02]);
%method_container = method_container.addMethod("RA_SAA_PID_alpha_005","RA-SAA-PID",["alpha",0.05]);
%method_container = method_container.addMethod("RA_SAA_PID_alpha_010","RA-SAA-PID",["alpha",0.10]);
%method_container = method_container.addMethod("RA_SAA_PID_alpha_020","RA-SAA-PID",["alpha",0.20]);
%method_container = method_container.addMethod("RA_SAA_PID_alpha_050","RA-SAA-PID",["alpha",0.50]);

method_container = method_container.addMethod("RA_SAA","RA-SAA");
method_container = method_container.addMethod("RA_SAA_PID","RA-SAA-PID");
method_container = method_container.addMethod("PID_CBF","PID-CBF");
method_container = method_container.addMethod("MPPI","MPPI");

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
    runMultiScenarioSimulation(method_container, scenario_name_detail,simulation_name_full,Nsim,Nplan,parameter_error_list(sim_cnt),robot_mass_list(sim_cnt),kill_all_visualize);
end