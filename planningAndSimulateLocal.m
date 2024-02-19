function [q,f,u,param_nominal] = planningAndSimulateLocal(u0,xd,Q,R,P,param_base,seed_list,lb,ub)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述
    
    [param_nominal,W_nominal] = system.makeUncertainty(1, param_base, true);
    Nt = param_nominal.Nt;
    Ns = length(seed_list);
    q = zeros(length(param_nominal.q0),Nt,Ns); % state variables
    f = zeros(length(u0(:,1)),Nt,Ns);  % force input
    u = zeros(length(u0(:,1)),Nt,Ns);
    cbf = system.CBF;
    opt = optimoptions(@quadprog, ...
    'Display','off');
    cbf = cbf.substituteParameters(param_nominal);  % CBF uses nominal model

    i = 0;
    for seed = seed_list
        i = i+1;
        [param, W] = system.makeUncertainty(seed, param_base, false);
        q(:,1,i) = param.q0;
        f(:,1,i) = param.f0;
        clear system.step
        if (param.q0(7,1)>param.q0(3,1))
            mode = 1;
        else
            mode = 2;
        end
        q_ddot = zeros(4,1);
        for t = 1:param_nominal.Nt-1    % system loop
            [u(:,t,i), ~] = system.ControllerPID(q(:,t,i), param.qd, 0, param, W(t+1)-W(t));
            if param.enable_CBF
                [A,b,h] = cbf.calculateConstraint(q(:,t,i),q_ddot,f(:,t,i));
                if h<0
                    disp("WARN: missing constraint h("+string(t)+")="+string(h))
                end
                du = quadprog(eye(4,4),[],A,b-A*u(:,t,i),[],[],[],[],[],opt);
                u(:,t,i) = u(:,t,i) + du;
            end
            [q(:,t+1,i), f(:,t+1,i), mode, q_ddot] = system.step(q(:,t,i), f(:,t,i), u(:,t,i), param, mode, 1, W(t+1)-W(t));
        end
    end
    %rng(seed_valid);
end

