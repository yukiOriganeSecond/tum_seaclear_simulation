
clear
clc

%%
folder_name_detail = "multi_test_2";
folder_name = "data/multi_scenario/"+folder_name_detail;
mkdir(folder_name+"/variables")
mkdir(folder_name+"/paths")
mkdir(folder_name+"/inputs")
%mkdir(folder_name);    % please make folder manualy

%% make scenario
scenario_setting_param = struct;
Nsc = 1;    % number of scenario
scenario_setting_param.number_of_scenario = Nsc;
scenario_setting_param.seed_length = 100;
scenario_setting_param.tether_speed = 0.3;
scenario_setting_param.robot_horizontal_speed = 0.5;
scenario_setting_param.termination_time_coefficient = 1.0;
scenario_setting_param.y0_limitation = [[-3 3]; [0 0]; [1 6]; [0 0]; [-3 3]; [0 0]];           % [x(0); dx(0); d(0); dd(0); X(0); dX(0)];
scenario_setting_param.yd_limitation = [[-3 3]; [0 0]; [1 6]; [0 0]; [-3 3]; [0 0]];            % [xd; dxd; dd; ddd; Xd; dXd];
scenario_setting_param.obstacle_limitation = [[-3 3]; [2 6]; [0.5 2]];                         % [xo, do, obs_size];
%scenario_setting_param.obstacle_limitation = [[-4 -4]; [3 3]; [1 1]]; 
scenario_setting_param.number_of_obstacles = 1;
scenario_setting_param.y0_yd_min_distance = 1.0;                                               % keep this distance between y(0) and yd
scenario_setting_param.obs_y_min_distance = 0.3;                                               % keep this distance between y(0),yd and obstacles
scenario_setting_param.max_sample_trial = 100;
scenario_setting_param.visualize_initial_condition = true;

make_scenario_list = 1:Nsc;  % if you want remake, please modify this list

scenario = makeScenario(make_scenario_list,scenario_setting_param);
save(folder_name+"/scenario_param.mat","scenario_setting_param","scenario")

layout = [1,Nsc];
%layout = [];
visual.plotScenarioCondition(make_scenario_list,scenario,folder_name,layout);


%% run simulation
load(folder_name+"/scenario_param.mat")
method_list = ["RA-SAA","RA-SAA-PID"];

Nsc = length(scenario);
for s = 1:Nsc % loop for scenario
    clear x
    seed_plan = scenario(s).seed_base_1(1:10);
    seed_simulate = scenario(s).seed_base_2(1:10);

    for method = method_list
        % common setting
        param_base = makeStandardParameters(method);
        theta_0 = atan2(scenario(s).y0(5)-scenario(s).y0(1), scenario(s).y0(3));
        r_0 = vecnorm([scenario(s).y0(5)-scenario(s).y0(1), scenario(s).y0(3)]);
        param_base = system.addParam(param_base,"q0",[theta_0;0;r_0;0;scenario(s).y0(5);0;r_0;0]);
        param_base = system.addParam(param_base,"xd",scenario(s).yd);
        param_base = system.addParam(param_base,"obs_pos",scenario(s).obs_pos,"Deterministic",[0.10 0.10 0.10]);
        param_base = system.addParam(param_base,"obs_size",scenario(s).obs_size,"Deterministic",0.1);
        Nt = scenario(s).termination_time/param_base.dt.average;
        param_base = system.addParam(param_base,"Nt",Nt,"Deterministic");

        % method depended setting & perform simulation
        if ismember(method, ["RA-SAA","RA-SAA-PID"])
            [q,f,u,param_nominal] = planningAndSimulateSAA(param_base,seed_plan,seed_simulate); % SAA method
        end
        % evaluation
        for i = 1:length(seed_simulate)
            x(:,:,i) = system.changeCoordinate(q(:,:,i),param_nominal);
        end
        max_energy_consumption = energyEvaluation(u(:,:,:),f(:,:,:),param_nominal);
        
        % save results
        save(folder_name+"/variables/scenario_"+sprintf("%03d",s)+"_"+method+".mat");
        t_vec = param_nominal.dt:param_nominal.dt:param_nominal.dt*param_nominal.Nt;
        visual.plotInputs(u,f,param_nominal,t_vec,[1,2;3,4],folder_name+"/inputs/scenario_"+sprintf("%03d",s)+"_"+method+"_",1:length(seed_simulate))
        visual.makeSnapsWithPoints(q,x,param_nominal,scenario(s),t_vec,folder_name+"/paths/scenario_"+sprintf("%03d",s)+"_"+method+"_",[1],1:length(seed_simulate));
    end
end
