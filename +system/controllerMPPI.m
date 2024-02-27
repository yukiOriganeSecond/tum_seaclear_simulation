
function [us_, F, face_infeasible] = controllerMPPI(qt, ft, u0s, param_nominal, param_plan_list, W_plan_list, fig)
    param_plan_list = param_nominal;    % ORIGINAL MPPI, USE NOMINAL MODEL TO PREDICT 

    N_input_sample = param_nominal.number_of_input_sample;
    N_model = length(param_plan_list);
    Nk = N_input_sample*N_model;

    epsilons = pagemtimes(repmat(sqrt(pinv(param_nominal.R)),[1,1,Nk]),randn(size(u0s,1),size(u0s,2),Nk));
    v = zeros(size(epsilons));
    S = zeros(1,Nk);
    x = zeros(length(param_nominal.xd),param_nominal.predict_steps,Nk+1);
    k=0;
    face_infeasible = true;
    for model_cnt = 1:N_model
        W = W_plan_list(model_cnt,:);
        W = repmat(W,1,ceil(param_nominal.predict_steps/param_nominal.Nt));
        for sample_cnt = 1:N_input_sample
            k = k+1;
            if (qt(7,1)>qt(3,1))
                mode = 1;
            else
                mode = 2;
            end
            q_ = zeros(length(param_plan_list(model_cnt).q0),param_plan_list(model_cnt).predict_steps);
            f_ = zeros(size(u0s,1),param_plan_list(model_cnt).predict_steps);
            q_(:,1) = qt;
            f_(:,1) = ft;
            if k<=(1-param_plan_list(model_cnt).alpha_MPPI)*N_input_sample
                vs_ = u0s + epsilons(:,:,k);
            else
                vs_ = epsilons(:,:,k);
            end
            v(:,:,k) = repelem(vs_,1,param_plan_list(model_cnt).input_prescale);
            for t_sample = 1:param_plan_list(model_cnt).predict_steps-1
                [q_(:,t_sample+1), f_(:,t_sample+1), mode] = system.step(q_(:,t_sample), f_(:,t_sample), v(:,t_sample,k), param_plan_list(model_cnt), mode, W(t_sample+1)-W(t_sample));
            end
            x(:,:,k) = system.changeCoordinate(q_,param_plan_list(model_cnt),param_nominal.xd);
            [S(k),violate_constraint] = evaluateStates(q_,param_plan_list(model_cnt).xd,param_plan_list(model_cnt));
            face_infeasible = face_infeasible & violate_constraint; % if a path does not violate, its feasible.
            S(k) = S(k) + sum(dot(param_plan_list(model_cnt).R*u0s(:,1:end-1),u0s(:,2:end)-vs_(:,2:end)))*param_nominal.dt*param_nominal.input_prescale;
        end
    end
    S(isnan(S)) = 1000^2;   % replace NaN as safficient large value
    rho = min(S);
    eta = sum(exp(-1/param_nominal.lambda*(S-rho)));
    w = exp(-1/param_nominal.lambda*(S-rho))/eta;
    us_ = sgolayfilt(permute(tensorprod(w,v,2,3),[2,3,1]),3,71,[],2);
    [q_,~,~] = system.steps(qt,repelem(us_(:,1:end-1),1,param_nominal.input_prescale),param_nominal,W,param_nominal.predict_steps);
    x(:,:,Nk+1) = system.changeCoordinate(q_,param_nominal,param_nominal.xd);
    F = [];
        
    if param_nominal.visual_capture
        figure(fig)
        for k = 1:Nk
            plot(x(1,:,k), x(3,:,k), 'b');
            hold on
        end
        axis ij
        plot(x(1,:,Nk+1), x(3,:,Nk+1), 'r');
        hold off
        ylim([-1,7])
        xlim([-4,4])
        if param_nominal.consider_collision
            for l = 1:size(param_nominal.obs_pos,2)
                % NOTICE: Here, obstacles should refer parameters for
                % simulation
                objpos(l,:) = [(param_nominal.obs_pos(:,l)-param_nominal.obs_size(1,l)).', (2*param_nominal.obs_size(:,l))*[1 1]];
                rectangle('Position',objpos(l,:),'Curvature',[1 1]);
            end
        end
        F = getframe;
    end
        
end
