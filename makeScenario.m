function param_ = makeScenario(make_scenario_list,scenario_setting)
%UNTITLED2 この関数の概要をここに記述
%   詳細説明をここに記述
    set_ = scenario_setting;
    param_ = struct;

    for s = make_scenario_list
        param_(s).seed_base_1 = randi(set_.seed_length,1,set_.seed_length);
        param_(s).seed_base_2 = randi(set_.seed_length,1,set_.seed_length);
        cnt_ = 0;
        while 1     % sample initial and target point
            y0_ = set_.y0_limitation(:,1) + rand(length(set_.y0_limitation),1).*(set_.y0_limitation(:,2)-set_.y0_limitation(:,1));
            yd_ = set_.yd_limitation(:,1) + rand(length(set_.yd_limitation),1).*(set_.yd_limitation(:,2)-set_.yd_limitation(:,1));
            if set_.vessel_target_position_to_robot == true
                yd_(5,1) = yd_(1,1);
            end
            if set_.vessel_initial_position_to_target == true
                y0_(5,1) = yd_(5,1);
            end
            if (vecnorm(y0_([1,3],1)-yd_([1,3],1))>set_.y0_yd_min_distance)
                break   % repeat sampling until y0 and yd satisfies min_length condition
            end
            cnt_ = cnt_+1;
            if cnt_>set_.max_sample_trial
                disp("WARN: Could not Find feasible y0 and yd")
                break;
            end
        end
        param_(s).y0 = y0_;
        param_(s).yd = yd_;
        param_(s).termination_time = max([ceil(abs(yd_(3,1)-y0_(3,1))/set_.tether_speed*set_.termination_time_coefficient), ceil(abs(yd_(1,1)-y0_(1,1))/set_.robot_horizontal_speed*set_.termination_time_coefficient), ceil(abs(yd_(5,1)-y0_(5,1))/set_.vessel_horizontal_speed*set_.termination_time_coefficient)]);
        param_(s).termination_time = param_(s).termination_time + (mod(param_(s).termination_time,2)==1);   % termination time should be odd number

        for obs_cnt = 1:set_.number_of_obstacles
            cnt_ = 0;
            while 1     % sample obstacle setting
                obs_ = set_.obstacle_limitation(:,1) + rand(length(set_.obstacle_limitation),1).*(set_.obstacle_limitation(:,2)-set_.obstacle_limitation(:,1));
                if (vecnorm(y0_([1,3],1)-obs_([1,2],1))-obs_(3,1)>set_.obs_y_min_distance)&&(vecnorm(yd_([1,3],1)-obs_([1,2],1))-obs_(3,1)>set_.obs_y_min_distance)
                    break   % repeat sampling until obs_pos satisfies min_length condition
                end
                cnt_ = cnt_+1;
                if cnt_>set_.max_sample_trial
                    disp("WARN: Could not Find feasible y0 and yd")
                    break;
                end
            end
            param_(s).obs_pos(:,obs_cnt) = obs_([1,2],1);
            param_(s).obs_size(1,obs_cnt) = obs_(3);
        end
    end

end