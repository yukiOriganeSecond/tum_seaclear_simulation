function [c,ceq] = uncertaintyConstraint(u,xd,Q,R,P,param_base,opt_cnt,seed_list)
%EVALUATEINPUT この関数の概要をここに記述
%   詳細説明をここに記述
    %persistent u_cnt
    %persistent U_r_list
    %if isempty(u_cnt)
    %    u_cnt = 0;
    %    U_r_list = zeros(3000,param.Nt);
    %end
    %u_cnt = u_cnt+1;
    dist = zeros(length(seed_list),param_base.Nt.average);
    dist_gnd = zeros(length(seed_list),param_base.Nt.average);  % distance from robot to ground
    i = 0;
    for seed = seed_list
        i = i+1;
        if param_base.low_side_controller.average == "none"
            [param,W] = system.makeUncertainty(seed, param_base);
            [q,~] = system.steps(param.q0,u,param,opt_cnt,W);           % simulate state variables
        else
            [param_nominal,W] = system.makeUncertainty(seed, param_base, true); % calc nominal parameters
            [q_nominal,~,~] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W); % calc nominal values
            [param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
            [q,~,~] = system.stepsFB(param_unc.q0,q_nominal,u,param_unc,opt_cnt,W); % calc nominal values
            param = param_unc;
        end
        x = system.changeCoordinate(q,param);   % output variables
        %x(1:2,:) = q(1:2,:);   % change output values
        %x(3:4,:) = q(7:8,:);
        dist(i,:) = vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size;
        dist_gnd(i,:) = param.ground_depth-x(3,:);
    end
    %t = -2:0.01:0.2;
    t = -0.2;
    alpha = 0.05;
    c_pre(1) = t+1/alpha/length(seed_list)*sum(max(max(-dist,[],2)-t,0),1);
    c_pre(2) = t+1/alpha/length(seed_list)*sum(max(max(-dist_gnd,[],2)-t,0),1);
    %c = min(c_pre); % inf t
    if param.consider_collision == true
        c = c_pre;
    else
        c = 0;
    end
    ceq = 0;%[];
end

