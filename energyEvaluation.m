function [eval_max] = energyEvaluation(u,f,q0,xd,Q,R,W,param,opt_cnt)
%EVALUATEINPUT この関数の概要をここに記述
%   詳細説明をここに記述
    persistent u_cnt
    persistent U_r_list
    if isempty(u_cnt)
        u_cnt = 0;
        U_r_list = zeros(3000,param.Nt);
    end
    u_cnt = u_cnt+1;
    %q = system.steps(q0,u,param,opt_cnt);           % simulate state variables
    %x = system.changeCoordinate(q,param);   % output variables
    L = zeros(1,size(f,3));                  % cost function at t
    for i = 1:size(f,3)
        %L(1,t) = sqrt(f(:,t).'*f(:,t));
        L(1,i) = sum(vecnorm(f(:,:,i),2,1));
    end
    eval_max = max(L(1,:))*param.dt;
    %U_r_list(u_cnt,:) = u(2,:);
end

