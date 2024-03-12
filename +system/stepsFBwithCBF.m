function [q,f,u,face_infeasible] = stepsFBwithCBF(q0,param,param_nominal,W,cbf,q_nominal,u_nominal)
    arguments
        q0      % initial state
        param   % parameters set
        param_nominal   % nominal parameter
        W       % Winner Process
        cbf     % cbf object
        q_nominal = zeros(size(q0,1),param.Nt)   % nominal states (target trajectory)
        u_nominal = zeros(length(param.f0),param.Nt)   % time series of control input (signal input)
    end
    opt = optimoptions(@quadprog, ...
    'Display','off');
    cbf = cbf.substituteParameters(param_nominal);  % CBF uses nominal model
    %cbf = cbf.substituteParameters(param);  % CBF uses real model

    if ~isfield(param,"low_side_controller")  % If not defined
        param.low_side_controller = "none"; % treated as FF system
    end
    q = zeros(length(q0),param.Nt); % state variables
    f = zeros(length(u_nominal(:,1)),param.Nt);  % force input
    q(:,1) = q0;
    f(:,1) = param.f0;
    clear system.step
    if (q0(7,1)>q0(3,1))
        mode = 1;
    else
        mode = 2;
    end
    u = u_nominal;  % initialuze u_use by u
    q_ddot = zeros(4,1);
    face_infeasible = false;
    
    if param.trajectory == "astar"
        q_nominal(:,:) = system.astarTargetPathPlan(param_nominal);
    end

    for t = 1:param.Nt-1
        if param.use_gravity_compensate
            u_nominal(:,t) = param_nominal.bar_m*param_nominal.g*[sin(q(1,t));0;-cos(q(1,t));0];
        end
        if param.trajectory == "heuristic"
            q_nominal(:,t) = (param.qd-param.q0)/param_nominal.Nt*t+param.q0;
        end
        if param.low_side_controller == "PID"
            [u(:,t), ~] = system.ControllerPID(q(:,t), q_nominal(:,t), u_nominal(:,t), param, W(t+1)-W(t), t==1);
        end
        if param.enable_CBF
            [A,b,h] = cbf.calculateConstraint(q(:,t)+param.sensing_noise*(W(t+1)-W(t)),q_ddot+q_ddot.*param.acc_noise*(W(t+1)-W(t)),f(:,t)+f(:,t).*param.force_noise_coeff*(W(t+1)-W(t)));
            if h<0
                disp("WARN: missing constraint h("+string(t)+")="+string(h))
            end
            du = quadprog(eye(4,4),[],A,b-A*u(:,t),[],[],param.lb-u(:,t),param.ub-u(:,t),[],opt);
            if size(du,1)~=size(u(:,t))
                %u(:,t,i) = 0;   % if no solution, u should be 0
                % if no solution, do not apply CBF
                face_infeasible = true;
                disp("WARN: cbf no solution")
            else
                u(:,t) = u(:,t) + du;
            end
        end
        [q(:,t+1),f(:,t+1),mode,q_ddot] = system.step(q(:,t),f(:,t),u(:,t),param,mode,W(t+1)-W(t));
    end
end