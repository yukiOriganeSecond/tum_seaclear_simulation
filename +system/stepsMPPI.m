
function [q,f,u,face_infeasible,F_] = stepsMPPI(param_sim, param_nominal, param_plan_list, W_sim, W_plan_list, fig)   % simulation loop
    Nt_ = param_nominal.Nt;
    q = zeros(length(param_nominal.q0),Nt_); % state variables
    f = zeros(length(param_nominal.u0(:,1)),Nt_);  % force input
    u = zeros(length(param_nominal.u0(:,1)),Nt_);
    q(:,1) = param_sim.q0;
    f(:,1) = param_sim.f0;
    u0 = repmat(param_nominal.u0,1,Nt_);
    us_pred = repmat(u0(:,1),1,param_nominal.predict_steps/param_nominal.input_prescale+1); % MPPI's result

    if (q(7,1)>q(3,1))
        mode = 1;
    else
        mode = 2;
    end
    face_infeasible = false;
    j = 0;
    F_(ceil(param_nominal.Nt/param_nominal.input_prescale)) = struct('cdata',[],'colormap',[]);
    for t_sys = 1:param_nominal.Nt-1    % system loop
        if (mod(t_sys,param_nominal.input_prescale) == 1) || (t_sys == 1) || (param_nominal.input_prescale == 1)
            j = j+1;
            if param_nominal.visual_capture
                % controller does not real model param_sim, but
                % param_nominal and param_plan
                [us_pred, F_(j), is_infeasible_] = system.controllerMPPI(q(:,t_sys), f(:,t_sys), us_pred, param_nominal, param_plan_list, W_plan_list, fig);
            else
                [us_pred, ~, is_infeasible_] = system.controllerMPPI(q(:,t_sys), f(:,t_sys), us_pred, param_nominal, param_plan_list, W_plan_list, fig);
            end
            face_infeasible = face_infeasible | is_infeasible_;   % if once face infeasible, its out.
            u(:,t_sys) = us_pred(:,1);
            us_pred(:,1:end-1) = us_pred(:,2:end);
            us_pred(:,end) = u0(:,end);
        else
            u(:,t_sys) = u(:,t_sys-1);
        end
        [q(:,t_sys+1), f(:,t_sys+1), mode, ~] = system.step(q(:,t_sys), f(:,t_sys), u(:,t_sys), param_sim, mode, W_sim(t_sys+1)-W_sim(t_sys));
    end
end