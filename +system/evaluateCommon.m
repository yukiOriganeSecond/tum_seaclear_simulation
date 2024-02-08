function [q,x,eval_result,grad,dist,dist_gnd] = evaluateCommon(us,xd,Q,R,P,param_base,opt_cnt,param_nominal,param_sets,W_nominal,W_sets)
    u = repelem(us,1,param_base.input_prescale.average); % insert missing section

    Ns = length(param_sets);
    q = zeros(length(param_base.q0.average),param_base.Nt.average,Ns);
    x = zeros(length(xd),param_base.Nt.average,Ns);
    dist = zeros(Ns,param_base.Nt.average);
    dist_gnd = zeros(Ns,param_base.Nt.average);  % distance from robot to ground
    eval_result = 0;
    grad = 0;

    if ~isfield(param_base,"low_side_controller")  % If not defined
        param_base = system.addParam(param_base,"low_side_controller","none","Deterministic");  % treated as FF systen
    end
    %[param_nominal,W] = system.makeUncertainty(seed_list(1), param_base, true); % calc nominal parameters
    [q_nominal,~,~] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W_nominal); % calc nominal values

    for i = 1:Ns
        %[param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
        if param_base.low_side_controller.average == "none"
            [q(:,:,i),~,u_use] = system.steps(param_sets(i).q0,u,param_sets(i),opt_cnt,W_sets(i,:)); % steps openloop
        else
            [q(:,:,i),~,u_use] = system.stepsFB(param_sets(i).q0,q_nominal,u,param_sets(i),opt_cnt,W_sets(i,:));  % steps with feedback
        end
        param = param_sets(i);
        x(:,:,i) = system.changeCoordinate(q(:,:,i),param);   % output variables
        dist(i,:) = vecnorm(x([1,3],:,i)-param.obs_pos,2,1)-param.obs_size;
        dist_gnd(i,:) = param.ground_depth-x(3,:,i);
        
        L = param_nominal.dt*dot(R*u_use(:,:),u_use(:,:)) + param_nominal.dt*dot(Q*(x(:,:,i)-xd(:,1)),(x(:,:,i)-xd(:,1)));
        eval_result = eval_result + sum(L(:,:))+(x(:,end,i)-xd(:,1)).'*P*(x(:,end,i)-xd(:,1));
        grad = grad + param_nominal.dt*(R*u_use(:,:));  % if Q and P = 0 only case
    end
end


