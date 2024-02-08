function [q,x,eval_result,grad,dist,dist_gnd] = evaluateCommon(u,xd,Q,R,P,param_base,opt_cnt,seed_list)

    Ns = length(seed_list);
    q = zeros(length(param_base.q0.average),param_base.Nt.average,Ns);
    x = zeros(length(xd),param_base.Nt.average,Ns);
    dist = zeros(length(seed_list),param_base.Nt.average);
    dist_gnd = zeros(length(seed_list),param_base.Nt.average);  % distance from robot to ground
    eval_result = 0;
    grad = 0;

    if ~isfield(param_base,"low_side_controller")  % If not defined
        param_base = system.addParam(param_base,"low_side_controller","none","Deterministic");  % treated as FF systen
    end
    [param_nominal,W] = system.makeUncertainty(seed_list(1), param_base, true); % calc nominal parameters
    [q_nominal,~,u_use] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W); % calc nominal values
    i=0;
    for seed = seed_list
        i = i+1;
        if param_base.low_side_controller.average == "none"
            q(:,:,i) = q_nominal;
        else
            [param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
            [q(:,:,i),~,u_use] = system.stepsFB(param_unc.q0,q_nominal,u,param_unc,opt_cnt,W); % calc nominal values
            param = param_unc;
        end
        x(:,:,i) = system.changeCoordinate(q(:,:,i),param);   % output variables
        dist(i,:) = vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size;
        dist_gnd(i,:) = param.ground_depth-x(3,:);
        
        L = param_nominal.dt*dot(R*u_use(:,:),u_use(:,:)) + param_nominal.dt*dot(Q*(x(:,:)-xd(:,1)),(x(:,:)-xd(:,1)));
        eval_result = eval_result + sum(L(:,:))+(x(:,end)-xd(:,1)).'*P*(x(:,end)-xd(:,1));
        grad = grad + param_nominal.dt*(R*u_use(:,:));  % if Q and P = 0 only case
    end
end


