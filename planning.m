function [u,fval] = planning(u0,xd,Q,R,P,param_base,opt_cnt,seed_list,lb,ub,options)
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
    eval_result = 0;
    grad = 0;
    
    [param_nominal,W_nominal] = system.makeUncertainty(1, param_base, true); % calc uncertained parameters
    i = 0;
    for seed = seed_list
        i = i+1;
        [param_sets(i),W_sets(i,:)] = system.makeUncertainty(seed, param_base, false);
    end

    [us,fval] = fmincon(fun,u0(:,1:param_base.input_prescale.average:Nt),[],[],[],[],param_base.enable_u.average.*repmat(lb,1,Nu),param_base.enable_u.average.*repmat(ub,1,Nu),cfun,options);
    u = repelem(us,1,param_base.input_prescale.average); % insert missing section

    function [eval_result_,grad_] = evalInput(us)
        if ~isequal(us,us_last)
            [q,x,eval_result,grad,dist,dist_gnd] = system.evaluateCommon(us,xd,Q,R,P,param_base,opt_cnt,param_nominal,param_sets,W_nominal,W_sets);
            us_last = us;
        end
        eval_result_ = eval_result/length(seed_list);
        grad_ = grad/length(seed_list);
    end

    function [c,ceq] = evalConstraint(us)
        if ~isequal(us,us_last)
            [q,x,eval_result,grad,dist,dist_gnd] = system.evaluateCommon(us,xd,Q,R,P,param_base,opt_cnt,param_nominal,param_sets,W_nominal,W_sets);
            us_last = us;
        end
        t = -0.2;
        alpha = 0.05;
        %c(1) = mean(vecnorm(x([1,3],end,:)-xd([1,3],1),2,1),3)-0.1;
        %c(2) = mean(vecnorm(x([2,4],end,:)-xd([2,4],1),2,1),3)-0.1;
        if param_base.consider_collision.average == true
            c(1) = t+1/alpha/length(seed_list)*sum(max(max(-dist,[],2)-t,0),1);
            c(2) = t+1/alpha/length(seed_list)*sum(max(max(-dist_gnd,[],2)-t,0),1);
        else
            c = 0;
        end
        ceq = 0;
    end
    
end

