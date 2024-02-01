function eval_result = evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list)
%EVALUATEINPUT この関数の概要をここに記述
%   詳細説明をここに記述
    %persistent u_cnt
    %persistent U_r_list
    %if isempty(u_cnt)
    %    u_cnt = 0;
    %    U_r_list = zeros(3000,param_base.Nt.average);
    %end
    %u_cnt = u_cnt+1;
    i = 0;
    eval_result = 0;
    for seed = seed_list
        [param,W] = system.makeUncertainty(seed, param_base);
        [q,f] = system.steps(param.q0,u,param,opt_cnt,W);           % simulate state variables
        x = system.changeCoordinate(q,param);   % output variables
        L = zeros(1,param.Nt);                  % cost function at t
        %x(1:2,:) = q(1:2,:);   % change output values
        %x(3:4,:) = q(7:8,:);
        for t = 1:param.Nt
            L(1,t) = u(:,t).'*R*u(:,t) + (x(:,t)-xd(:,1)).'*Q*(x(:,t)-xd(:,1));
        end
        eval_result = eval_result + sum(L(1,:))*param.dt+(x(:,end)-xd(:,1)).'*P*(x(:,end)-xd(:,1));
    end
    eval_result = eval_result/length(seed_list);
    %U_r_list(u_cnt,:) = u(2,:);
end

