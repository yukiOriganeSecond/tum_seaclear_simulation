    
    target_folder_name = "multiple_0301_b/mass_250";
    save_folder_name = "data/multi_scenario/"+target_folder_name;

    kill_all_visualize = true;
    method_container = MethodContainer;

    method_container = method_container.addMethod("RA_SAA","RA-SAA");
    method_container = method_container.addMethod("RA_SAA_PID","RA-SAA-PID");
    method_container = method_container.addMethod("PID_CBF","PID-CBF");
    method_container = method_container.addMethod("MPPI","MPPI");

    Nsc = 50;                                        % number of scenario
    Nm = method_container.getNumberOfMethods();     % number of method
    Nsim = 20;
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
        for method_index = 1:Nm
            method_name = method_container.getMethodName(method_index);
            load(save_folder_name+"/variables/scenario_"+sprintf("%03d",s)+"_"+method_name+".mat", "q", "param_sim", "scenario", "u", "f", "param_nominal", "seed_simulate")
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
            %save(save_folder_name+"/variables/scenario_"+sprintf("%03d",s)+"_"+method_name+".mat");
        end
    end
    toc
    save(save_folder_name+"/results.mat");
