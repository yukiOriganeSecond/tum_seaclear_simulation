function [u,fval,t_end] = planning(u0,xd,Q,R,P,param_base,seed_list,lb,ub,options)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述
    
    us_last = [];

    fun = @evalInput;
    cfun = @evalConstraint;

    Ns = length(seed_list);
    Nu = param_base.Nt.average/param_base.input_prescale.average;
    Nt = param_base.Nt.average;
    q = zeros(length(param_base.q0.average),param_base.Nt.average,Ns);
    x = zeros(length(xd),param_base.Nt.average,Ns);
    dist = zeros(length(seed_list),param_base.Nt.average);
    dist_gnd = zeros(length(seed_list),param_base.Nt.average);  % distance from robot to ground
    dist_right = zeros(length(seed_list),param_base.Nt.average);  % distance from robot to ground
    eval_result = 0;
    grad = 0;
    
    [param_nominal,W_nominal] = system.makeUncertainty(1, param_base, true); % calc uncertained parameters
    i = 0;
    for seed = seed_list
        i = i+1;
        [param_sets(i),W_sets(i,:)] = system.makeUncertainty(seed, param_base, false);
    end

    [ust,fval,~,output] = fmincon(fun,[u0(:,1:param_base.input_prescale.average:Nt),[-0.2;0;0;0]],[],[],[],[],[repmat(lb,1,Nu),[-inf;0;0;0]],[repmat(ub,1,Nu),[+inf;0;0;0]],cfun,options);
    if (output.constrviolation > 1e-6) && (~isempty(output.bestfeasible))
        ust = output.bestfeasible.x;
        fval = output.bestfeasible.fval;
        disp("WARN: use best feasible point")
    end
    u = repelem(ust(:,1:end-1),1,param_base.input_prescale.average); % insert missing section
    t_end = ust(1,end);

    function [eval_result_,grad_] = evalInput(ust)
        us = ust(:,1:end-1);
        %t = ust(end,1);
        if ~isequal(us,us_last)
            [q,x,eval_result,grad,dist,dist_gnd,dist_right] = system.evaluateCommon(us,xd,Q,R,P,param_base,param_nominal,param_sets,W_nominal,W_sets);
            us_last = us;
        end
        eval_result_ = eval_result/length(seed_list);
        grad_ = grad;%/length(seed_list);
    end

    function [c,ceq] = evalConstraint(ust)
        us = ust(:,1:end-1);
        t = ust(1,end);
        if ~isequal(us,us_last)
            [q,x,eval_result,grad,dist,dist_gnd,dist_right] = system.evaluateCommon(us,xd,Q,R,P,param_base,param_nominal,param_sets,W_nominal,W_sets);
            us_last = us;
        end
        
        c = zeros(1,5);
        if param_base.consider_collision.average == true
            %c(1) = param_nominal.t+1/param_nominal.alpha/length(seed_list)*sum(max(max(-dist,[],2)-param_nominal.t,0),1);
            %c(2) = param_nominal.t+1/param_nominal.alpha/length(seed_list)*sum(max(max(-dist_gnd,[],2)-param_nominal.t,0),1);
            t = -1:0.001:0;
            c(1) = min(t+1/param_nominal.alpha/length(seed_list)*sum(max(max(-dist,[],2)-t,0),1));
            c(2) = min(t+1/param_nominal.alpha/length(seed_list)*sum(max(max(-dist_gnd,[],2)-t,0),1));
        end
        if param_base.right_side_constraints.average == true
            c(3) = param_nominal.t+1/param_nominal.alpha/length(seed_list)*sum(max(max(-dist_right,[],2)-param_nominal.t,0),1);
        end
        if size(xd,1)==4
            c(4) = mean(vecnorm(x([1,3],end,:)-xd([1,3],1),2,1),3)-param_nominal.equality_slack(1);
            c(5) = mean(vecnorm(x([2,4],end,:)-xd([2,4],1),2,1),3)-param_nominal.equality_slack(2);
        else
            c(4) = mean(vecnorm(x([1,3,5],end,:)-xd([1,3,5],1),2,1),3)-param_nominal.equality_slack(1);
            c(5) = mean(vecnorm(x([2,4,6],end,:)-xd([2,4,6],1),2,1),3)-param_nominal.equality_slack(2);
        end
        ceq = 0;
    end
    
end

