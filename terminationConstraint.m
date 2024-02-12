function [c,ceq] = terminationConstraint(u,xd,Q,R,P,param_base,opt_cnt,seed_list)
%EVALUATEINPUT この関数の概要をここに記述
%   詳細説明をここに記述
    %persistent u_cnt
    %persistent U_r_list
    %if isempty(u_cnt)
    %    u_cnt = 0;
    %    U_r_list = zeros(3000,param.Nt);
    %end
    %u_cnt = u_cnt+1;
    i = 0;
    for seed = seed_list
        i = i+1;
        if param_base.low_side_controller.average == "none"
            [param,W] = system.makeUncertainty(seed, param_base);
            [q,~] = system.steps(param.q0,u,param,opt_cnt,W);           % simulate state variables
        else
            [param_nominal,W] = system.makeUncertainty(seed, param_base, true); % calc nominal parameters
            [q_nominal,~,~] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W); % calc nominal values
            [param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
            [q,~,~] = system.stepsFB(param_unc.q0,q_nominal,u,param_unc,opt_cnt,W); % calc nominal values
            param = param_unc;
        end
        x = system.changeCoordinate(q,param);   % output variables
        %x(1:2,:) = q(1:2,:);   % change output values
        %x(3:4,:) = q(7:8,:);
    end
    %t = -2:0.01:0.2;

    c(1) = (x([1,3],end)-xd([1,3],1)).'*(x([1,3],end)-xd([1,3],1))-0.01;
    c(2) = (x([2,4],end)-xd([2,4],1)).'*(x([2,4],end)-xd([2,4],1))-0.01;
    ceq = 0;%[];
end

