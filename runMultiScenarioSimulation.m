
function runMultiScenarioSimulation(method_container, scenario_name, simulation_name, Nsim, Nplan, param_error, kill_all_visualize)
    arguments
        method_container
        scenario_name
        simulation_name
        Nsim
        Nplan
        param_error
        kill_all_visualize = true
    end

    load("scenarios/"+scenario_name+"/scenario_param.mat","scenario")
    save_folder_name = "data/multi_scenario/"+simulation_name;
    mkdir(save_folder_name+"/variables")
    mkdir(save_folder_name+"/paths")
    mkdir(save_folder_name+"/inputs")
    
    %method_list = ["RA-SAA","RA-SAA-PID"];
    %method_list = ["RA-SAA"];
    Nsc = length(scenario);                       % number of scenario
    Nm = method_container.getNumberOfMethods();   % number of method
    %Nplan = 10;                                   % number of sample for planning
    %Nsim = 10;                                    % number of sample for simulation
    
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
        for method_index = 1:Nm
            % common setting
            base_method = method_container.getBaseMethod(method_index);
            param_base = makeStandardParameters(base_method);
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
            param_base.m.error = param_error;
            param_base.bar_m.error = param_error;
            param_base.mu_theta.error = param_error;
            param_base.mu_r.error = param_error;
            param_base = method_container.subsMethodParameters(method_index,param_base);
            % method depended setting & perform simulation
            if ismember(base_method, ["RA-SAA","RA-SAA-PID"])
                if kill_all_visualize == true
                    %param_base = system.addParam(param_base,"opt_Display",'none',"Deterministic");
                    param_base = system.addParam(param_base,"opt_PlotFcn",[],"Deterministic");
                end
                param_base = method_container.subsMethodParameters(method_index,param_base);
                [q,f,u,param_nominal,param_sim,find_feasible_solution] = planningAndSimulateSAA(param_base,seed_plan,seed_simulate); % SAA method
                face_infeasible_solution(:,method_index,s) = ~find_feasible_solution;
            end
            if ismember(base_method, ["PID-CBF"])
                [q,f,u,param_nominal,param_sim,find_feasible_solution] = planningAndSimulateLocal(param_base,seed_simulate); % Local method
                face_infeasible_solution(:,method_index,s) = ~find_feasible_solution;
            end
            if ismember(base_method, ["MPPI"])
                if param_base.predict_steps.average > Nt
                    param_base.predict_steps.average = Nt;
                end
                if kill_all_visualize == true
                    param_base = system.addParam(param_base,"visual_capture",false,"Deterministic");
                end
                param_base = system.addParam(param_base,"predict_steps",200,"Deterministic");
                [q,f,u,param_nominal,param_sim,find_feasible_solution,~] = planningAndSimulateMPPI(param_base,seed_plan,seed_simulate); % MPPI method
                face_infeasible_solution(:,method_index,s) = ~find_feasible_solution;
            end
            % evaluation
            for i = 1:length(seed_simulate)
                dist_ = NaN;
                x(:,:,i) = system.changeCoordinate(q(:,:,i),param_nominal);
                for j = 1:size(scenario(s).obs_pos,2)
                    dist_ = min([dist_, vecnorm(x([1,3],:,i)-param_sim(i).obs_pos(:,j),2,1)-param_sim(i).obs_size(:,j)]);
                end
                minimum_collision_torelance(i,method_index,s) = dist_;
            end
            if size(x,1)==4          
                final_target_error_pos(:,method_index,s) = permute(vecnorm(x([1,3],end,:)-scenario(s).yd([1,3],1),2,1),[3,1,2]);
                final_target_error_vel(:,method_index,s) = permute(vecnorm(x([2,4],end,:)-scenario(s).yd([2,4],1),2,1),[3,1,2]);
            else
                final_target_error_pos(:,method_index,s) = permute(vecnorm(x([1,3,5],end,:)-scenario(s).yd([1,3,5],1),2,1),[3,1,2]);
                final_target_error_vel(:,method_index,s) = permute(vecnorm(x([2,4,6],end,:)-scenario(s).yd([2,4,6],1),2,1),[3,1,2]);
            end
            [~,energy_consumption(:,method_index,s)] = energyEvaluation(u(:,:,:),f(:,:,:),param_nominal);
            
            % save each results
            method_name = method_container.getMethodName(method_index);
            save(save_folder_name+"/variables/scenario_"+sprintf("%03d",s)+"_"+method_name+".mat");
            t_vec = param_nominal.dt:param_nominal.dt:param_nominal.dt*param_nominal.Nt;
            if ~kill_all_visualize
                visual.plotInputs(u,f,param_nominal,t_vec,[1,2;3,4],save_folder_name+"/inputs/scenario_"+sprintf("%03d",s)+"_"+method_name+"_",1:length(seed_simulate))
                visual.makeSnapsWithPoints(q,x,param_nominal,scenario(s),t_vec,save_folder_name+"/paths/scenario_"+sprintf("%03d",s)+"_"+method_name+"_",[1],1:length(seed_simulate));
            end
            close all   % once close all figure
        end
    end
    toc
    save(save_folder_name+"/results.mat");
end