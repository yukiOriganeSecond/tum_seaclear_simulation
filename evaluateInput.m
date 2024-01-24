function eval_result = evaluateInput(u,q0,xd,Q,R,W,param,opt_cnt)
%EVALUATEINPUT この関数の概要をここに記述
%   詳細説明をここに記述
    persistent u_cnt
    persistent U_r_list
    if isempty(u_cnt)
        u_cnt = 0;
        U_r_list = zeros(3000,param.Nt);
    end
    u_cnt = u_cnt+1;
    q = system.steps(q0,u,param,opt_cnt);           % simulate state variables
    x = system.changeCoordinate(q,param);   % output variables
    L = zeros(1,param.Nt);                  % cost function at t
    for t = 1:param.Nt
        L(1,t) = u(:,t).'*R*u(:,t) + (x(:,t)-xd(:,1)).'*Q*(x(:,t)-xd(:,1));
    end
    eval_result = sum(L(1,:))*param.dt+(x(:,end)-xd(:,1)).'*W*(x(:,end)-xd(:,1));
    %U_r_list(u_cnt,:) = u(2,:);
end

