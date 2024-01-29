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
    i = 0;
    for seed = seed_list
        i = i+1;
        [param,W] = system.makeUncertainty(seed, param_base);
        q = system.steps(param.q0,u,param,opt_cnt,W);           % simulate state variables
        x = system.changeCoordinate(q,param);   % output variables
        %x(1:2,:) = q(1:2,:);   % change output values
        %x(3:4,:) = q(7:8,:);
        dist(i,:) = vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size;
    end
    %t = -5.2:0.1:0.5;
    t = -0.2;
    alpha = 0.05;
    c_pre = t+1/alpha/length(seed_list)*sum(max(max(-dist,[],2)-t,0),1);
    %c = min(c_pre); % inf t
    c = c_pre;
    ceq = 0;%[];
end

