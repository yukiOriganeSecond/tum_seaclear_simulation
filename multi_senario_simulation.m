
clear
clc

%%
folder_name_detail = "multi_test_4";
folder_name = "data/multi_scenario/"+folder_name_detail;
mkdir(folder_name+"/variables")
mkdir(folder_name+"/paths")
mkdir(folder_name+"/inputs")

kill_all_visualize = false;            % if true, all visualizing are killed
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

layout = [1,Nsc];
%layout = [];
if ~kill_all_visualize
    visual.plotScenarioCondition(make_scenario_list,scenario,folder_name,layout);
end

%% run simulation
load(folder_name+"/scenario_param.mat")
%method_list = ["RA-SAA","RA-SAA-PID"];
method_list = ["RA-SAA","RA-SAA-PID","PID-CBF","MPPI"];
%method_list = ["RA-SAA"];
Nsc = length(scenario);     % number of scenario
Nm = length(method_list);   % number of method
Nplan = 10;                 % number of sample for planning
Nsim = 10;                  % number of sample for simulation

energy_consumption = zeros(Nsim, Nm, Nsc); % input energy
face_infeasible_solution = zeros(Nsim, Nm, Nsc);
minimum_collision_torelance = zeros(Nsim, Nm, Nsc);
final_target_error_pos = zeros(Nsim, Nm, Nsc);
final_target_error_vel = zeros(Nsim, Nm, Nsc);

tic
for s = 1:Nsc % loop for scenario
    clear x
    seed_plan = scenario(s).seed_base_1(1:Nplan);
    seed_simulate = scenario(s).seed_base_2(1:Nsim);
    cnt_method = 0;
    for method = method_list
        cnt_method = cnt_method+1;
        % common setting
        param_base = makeStandardParameters(method);
        theta_0 = atan2(scenario(s).y0(5)-scenario(s).y0(1), scenario(s).y0(3));
        r_0 = vecnorm([scenario(s).y0(5)-scenario(s).y0(1), scenario(s).y0(3)]);
        param_base = system.addParam(param_base,"q0",[theta_0;0;r_0;0;scenario(s).y0(5);0;r_0;0]);
        theta_d = atan2(scenario(s).yd(5)-scenario(s).yd(1), scenario(s).yd(3));
        r_d = vecnorm([scenario(s).yd(5)-scenario(s).yd(1), scenario(s).yd(3)]);
        param_base = system.addParam(param_base,"qd",[theta_d;0;r_d;0;scenario(s).yd(5);0;r_d;0]);
        param_base = system.addParam(param_base,"xd",scenario(s).yd);
        param_base = system.addParam(param_base,"obs_pos",scenario(s).obs_pos,"Deterministic",[0.10 0.10 0.10]);
        param_base = system.addParam(param_base,"obs_size",scenario(s).obs_size,"Deterministic",0.1);
        Nt = scenario(s).termination_time/param_base.dt.average;
        param_base = system.addParam(param_base,"Nt",Nt,"Deterministic");

        % method depended setting & perform simulation
        if ismember(method, ["RA-SAA","RA-SAA-PID"])
            if kill_all_visualize == true
                %param_base = system.addParam(param_base,"opt_Display",'none',"Deterministic");
                param_base = system.addParam(param_base,"opt_PlotFcn",[],"Deterministic");
            end
            [q,f,u,param_nominal,param_sim,find_feasible_solution] = planningAndSimulateSAA(param_base,seed_plan,seed_simulate); % SAA method
            face_infeasible_solution(:,cnt_method,s) = ~find_feasible_solution;
        end
        if ismember(method, ["PID-CBF"])
            [q,f,u,param_nominal,param_sim,find_feasible_solution] = planningAndSimulateLocal(param_base,seed_simulate); % Local method
            face_infeasible_solution(:,cnt_method,s) = ~find_feasible_solution;
        end
        if ismember(method, ["MPPI"])
            if param_base.predict_steps.average > Nt
                param_base.predict_steps.average = Nt;
            end
            if kill_all_visualize == true
                param_base = system.addParam(param_base,"visual_capture",false,"Deterministic");
            end
            param_base = system.addParam(param_base,"predict_steps",200,"Deterministic");
            [q,f,u,param_nominal,param_sim,find_feasible_solution,~] = planningAndSimulateMPPI(param_base,seed_plan,seed_simulate); % MPPI method
            face_infeasible_solution(:,cnt_method,s) = ~find_feasible_solution;
        end
        % evaluation
        for i = 1:length(seed_simulate)
            dist_ = NaN;
            x(:,:,i) = system.changeCoordinate(q(:,:,i),param_nominal);
            for j = 1:size(scenario(s).obs_pos,2)
                dist_ = min([dist_, vecnorm(x([1,3],:,i)-param_sim(i).obs_pos(:,j),2,1)-param_sim(i).obs_size(:,j)]);
            end
            minimum_collision_torelance(i,cnt_method,s) = dist_;
        end
        if size(x,1)==4          
            final_target_error_pos(:,cnt_method,s) = permute(vecnorm(x([1,3],end,:)-scenario(s).yd([1,3],1),2,1),[3,1,2]);
            final_target_error_vel(:,cnt_method,s) = permute(vecnorm(x([2,4],end,:)-scenario(s).yd([2,4],1),2,1),[3,1,2]);
        else
            final_target_error_pos(:,cnt_method,s) = permute(vecnorm(x([1,3,5],end,:)-scenario(s).yd([1,3,5],1),2,1),[3,1,2]);
            final_target_error_vel(:,cnt_method,s) = permute(vecnorm(x([2,4,6],end,:)-scenario(s).yd([2,4,6],1),2,1),[3,1,2]);
        end
        [~,energy_consumption(:,cnt_method,s)] = energyEvaluation(u(:,:,:),f(:,:,:),param_nominal);
        
        % save each results
        save(folder_name+"/variables/scenario_"+sprintf("%03d",s)+"_"+method+".mat");
        t_vec = param_nominal.dt:param_nominal.dt:param_nominal.dt*param_nominal.Nt;
        if ~kill_all_visualize
            visual.plotInputs(u,f,param_nominal,t_vec,[1,2;3,4],folder_name+"/inputs/scenario_"+sprintf("%03d",s)+"_"+method+"_",1:length(seed_simulate))
            visual.makeSnapsWithPoints(q,x,param_nominal,scenario(s),t_vec,folder_name+"/paths/scenario_"+sprintf("%03d",s)+"_"+method+"_",[1],1:length(seed_simulate));
        end
        close all   % once close all figure
    end
end
toc
save(folder_name+"/results.mat");

%% analysis
if ~kill_all_visualize
    color_base = ["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30"];
    visual.plotMultiAverageMax(energy_consumption,color_base,method_list,"energy consumption",folder_name)
    visual.plotMultiAverageMax(final_target_error_pos,color_base,method_list,"termination position error",folder_name,"max",0.3,"slack variable")
    visual.plotMultiAverageMax(minimum_collision_torelance,color_base,method_list,"collision torelance",folder_name,"min",0,"collision")
    visual.plotMultiAverageMax(mean(minimum_collision_torelance<0,1),color_base,method_list,"collision rate",folder_name)
    
    visual.plotMultiBarGraph(method_list,mean(mean(face_infeasible_solution,1),3),"infeasible ratio",folder_name)
end