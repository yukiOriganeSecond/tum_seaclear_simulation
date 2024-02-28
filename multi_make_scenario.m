
clear
clc

%%
scenario_name_detail = "path_20";
folder_name = "scenarios/"+scenario_name_detail;
mkdir(folder_name)
kill_all_visualize = false;            % if true, all visualizing are killed
%mkdir(folder_name);    % please make folder manualy

%% make scenario
scenario_setting_param = struct;
Nsc = 20;    % number of scenario
scenario_setting_param.number_of_scenario = Nsc;
scenario_setting_param.seed_length = 100;
scenario_setting_param.tether_speed = 0.3;
scenario_setting_param.robot_horizontal_speed = 0.5;
scenario_setting_param.termination_time_coefficient = 1.0;
scenario_setting_param.y0_limitation = [[-3 3]; [0 0]; [1 6]; [0 0]; [-3 3]; [0 0]];           % [x(0); dx(0); d(0); dd(0); X(0); dX(0)];
scenario_setting_param.yd_limitation = [[-3 3]; [0 0]; [1 6]; [0 0]; [-3 3]; [0 0]];            % [xd; dxd; dd; ddd; Xd; dXd];
scenario_setting_param.obstacle_limitation = [[-3 3]; [2 6]; [0.5 2]];                         % [xo, do, obs_size];
scenario_setting_param.vessel_initial_position_to_target = false;    % false
scenario_setting_param.vessel_target_position_to_robot = false;      % false
%scenario_setting_param.obstacle_limitation = [[-4 -4]; [3 3]; [1 1]]; 
scenario_setting_param.number_of_obstacles = 1;
scenario_setting_param.y0_yd_min_distance = 1.0;                                               % keep this distance between y(0) and yd
scenario_setting_param.obs_y_min_distance = 0.3;                                               % keep this distance between y(0),yd and obstacles
scenario_setting_param.max_sample_trial = 100;
scenario_setting_param.visualize_initial_condition = true;

make_scenario_list = 1:Nsc;  % if you want remake, please modify this list

scenario = makeScenario(make_scenario_list,scenario_setting_param);
save(folder_name+"/scenario_param.mat","scenario_setting_param","scenario")

layout = [4,5];
%layout = [];
if ~kill_all_visualize
    visual.plotScenarioCondition(make_scenario_list,scenario,folder_name,layout);
end

