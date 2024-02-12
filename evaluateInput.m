function [eval_result,grad] = evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list)
%EVALUATEINPUT この関数の概要をここに記述
%   詳細説明をここに記述
    %persistent u_cnt
    %persistent U_r_list
    %if isempty(u_cnt)
    %    u_cnt = 0;
    %    U_r_list = zeros(3000,param_base.Nt.average);
    %end
    %u_cnt = u_cnt+1;
    if ~isfield(param_base,"low_side_controller")  % If not defined
        param_base = system.addParam(param_base,"low_side_controller","none","Deterministic");  % treated as FF systen
    end
    i = 0;
    eval_result = 0;
    grad = 0;
    for seed = seed_list
        if param_base.low_side_controller.average == "none"
            [param_nominal,W] = system.makeUncertainty(seed, param_base);
            [q,~,u_use] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W);           % simulate state variables
        else
            [param_nominal,W] = system.makeUncertainty(seed, param_base, true); % calc nominal parameters
            [q_nominal,~,~] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W); % calc nominal values
            [param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
            [q,~,u_use] = system.stepsFB(param_unc.q0,q_nominal,u,param_unc,opt_cnt,W); % calc nominal values
        end
        x = system.changeCoordinate(q,param_nominal);   % output variables
        %L = zeros(1,param_nominal.Nt);                  % cost function at t
        %x(1:2,:) = q(1:2,:);   % change output values
        %x(3:4,:) = q(7:8,:);
        %for t = 1:param_nominal.Nt
        %    L(1,t) = u_use(:,t).'*R*u_use(:,t) + (x(:,t)-xd(:,1)).'*Q*(x(:,t)-xd(:,1));
        %end
        L = param_nominal.dt*dot(R*u_use(:,:),u_use(:,:)) + param_nominal.dt*dot(Q*(x(:,:)-xd(:,1)),(x(:,:)-xd(:,1)));
        eval_result = eval_result + sum(L(:,:))+(x(:,end)-xd(:,1)).'*P*(x(:,end)-xd(:,1));
        grad = grad + param_nominal.dt*(R*u_use(:,:));  % if Q and P = 0 only case
    end
    eval_result = eval_result/length(seed_list);
    grad = grad/length(seed_list);
    %U_r_list(u_cnt,:) = u(2,:);
end

