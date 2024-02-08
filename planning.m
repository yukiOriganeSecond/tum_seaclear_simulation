function [u,fval] = planning(u0,xd,Q,R,P,param_base,opt_cnt,seed_list,lb,ub,options)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述
    
    u_last = [];

    fun = @evalInput;
    cfun = @evalConstraint;

    Ns = length(seed_list);
    q = zeros(length(param_base.q0.average),param_base.Nt.average,Ns);
    x = zeros(length(xd),param_base.Nt.average,Ns);
    dist = zeros(length(seed_list),param_base.Nt.average);
    dist_gnd = zeros(length(seed_list),param_base.Nt.average);  % distance from robot to ground
    eval_result = 0;
    grad = 0;

    [u,fval] = fmincon(fun,u0,[],[],[],[],param_base.enable_u.average.*lb,param_base.enable_u.average.*ub,cfun,options);

    function [eval_result_,grad_] = evalInput(u)
        if ~isequal(u,u_last)
            [q,x,eval_result,grad,dist,dist_gnd] = system.evaluateCommon(u,xd,Q,R,P,param_base,opt_cnt,seed_list);
            u_last = u;
        end
        eval_result_ = eval_result/length(seed_list);
        grad_ = grad/length(seed_list);
    end

    function [c,ceq] = evalConstraint(u)
        if ~isequal(u,u_last)
            [q,x,eval_result,grad,dist,dist_gnd] = system.evaluateCommon(u,xd,Q,R,P,param_base,opt_cnt,seed_list);
            u_last = u;
        end
        t = -0.2;
        alpha = 0.05;
        c(1) = (x([1,3],end)-xd([1,3],1)).'*(x([1,3],end)-xd([1,3],1))-0.01;
        c(2) = (x([2,4],end)-xd([2,4],1)).'*(x([2,4],end)-xd([2,4],1))-0.01;
        if param_base.consider_collision.average == true
            c(3) = t+1/alpha/length(seed_list)*sum(max(max(-dist,[],2)-t,0),1);
            c(4) = t+1/alpha/length(seed_list)*sum(max(max(-dist_gnd,[],2)-t,0),1);
        end
        ceq = 0;
    end
    
end

