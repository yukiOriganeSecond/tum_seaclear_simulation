function [q,f,u,param_valid,F] = planningAndSimulateMPPI(xd,param_base,seed_list_sample,seed_valid)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述

    [param_valid,W_valid] = system.makeUncertainty(seed_valid, param_base, false);
    i = 0;
    for seed = seed_list_sample
        i = i+1;
        [param_sets(i), W_sets(i,:)] = system.makeUncertainty(seed, param_base, false);
    end
    rng(seed_valid);
    Nt = param_valid.Nt;
    q = zeros(length(param_valid.q0),Nt); % state variables
    f = zeros(length(param_valid.u0(:,1)),Nt);  % force input
    u = zeros(length(param_valid.u0(:,1)),Nt);
    q(:,1) = param_valid.q0;
    f(:,1) = param_valid.f0;
    u0 = repmat(param_valid.u0,1,param_valid.Nt);
    us_pred = repmat(u0(:,1),1,param_valid.predict_steps/param_valid.input_prescale+1);
    
    j = 0;
    F(ceil(param_valid.Nt/param_valid.input_prescale)) = struct('cdata',[],'colormap',[]);
    for t_sys = 1:param_valid.Nt-1    % system loop
        if (mod(t_sys,param_valid.input_prescale) == 1) || (t_sys == 1) || (param_valid.input_prescale == 1)
            j = j+1;
            if param_valid.visual_capture
                [us_pred, F(j)] = ControllerMPPI(t_sys, q(:,t_sys), f(:,t_sys), us_pred);
            else
                [us_pred, ~] = ControllerMPPI(t_sys, q(:,t_sys), f(:,t_sys), us_pred);
            end
            u(:,t_sys) = us_pred(:,1);
            us_pred(:,1:end-1) = us_pred(:,2:end);
            us_pred(:,end) = u0(:,end);
        else
            u(:,t_sys) = u(:,t_sys-1);
        end
        [q(:,t_sys+1), f(:,t_sys+1), mode, ~] = system.step(q(:,t_sys), f(:,t_sys), u(:,t_sys), param_valid, mode, W_valid(t_sys+1)-W_valid(t_sys));
    end

    function [us_, F] = ControllerMPPI(~, qt, ft, u0s)
        epsilons = pagemtimes(repmat(sqrt(pinv(param_valid.R)),[1,1,length(param_sets)]),randn(size(u0s,1),size(u0s,2),length(param_sets)));
        v = zeros(size(epsilons));
        S = zeros(1,length(param_sets));
        x = zeros(length(xd),param_valid.predict_steps,length(param_sets)+1);
        for k = 1:length(param_sets)
            if (qt(7,1)>qt(3,1))
                mode = 1;
            else
                mode = 2;
            end
            q_ = zeros(length(param_sets(k).q0),param_sets(k).predict_steps);
            f_ = zeros(size(u0s,1),param_sets(k).predict_steps);
            q_(:,1) = qt;
            f_(:,1) = ft;
            alpha = 0;        % debug
            if k<=(1-param_sets(k).alpha_MPPI)*length(param_sets)
                vs_ = u0s + epsilons(:,:,k);
            else
                vs_ = epsilons(:,:,k);
            end
            v(:,:,k) = repelem(vs_,1,param_sets(k).input_prescale);
            for t_sample = 1:param_sets(k).predict_steps-1
                [q_(:,t_sample+1), f_(:,t_sample+1), mode] = system.step(q_(:,t_sample), f_(:,t_sample), v(:,t_sample,k), param_sets(k), mode, W_sets(k,t_sample+1)-W_sets(k,t_sample));
            end
            x(:,:,k) = system.changeCoordinate(q_,param_sets(k),xd);
            S(1,k) = evaluateStates(q_,xd,param_sets(k));
            S(1,k) = S(1,k) + sum(dot(param_sets(k).R*u0s(:,1:end-1),u0s(:,2:end)-vs_(:,2:end)))*param_sets(k).dt*param_sets(k).input_prescale;
        end
        S(isnan(S)) = 1000^2;   % replace NaN as safficient large value
        rho = min(S);
        eta = sum(exp(-1/param_valid.lambda*(S-rho)));
        w = exp(-1/param_valid.lambda*(S-rho))/eta;
        %us_ = u0s + sgolayfilt(permute(tensorprod(w,epsilons,2,3),[2,3,1]),3,71,[],2);
        us_ = sgolayfilt(permute(tensorprod(w,v,2,3),[2,3,1]),3,71,[],2);
        [q_,~,~] = system.steps(qt,repelem(us_(:,1:end-1),1,param_valid.input_prescale),param_valid,W_valid,param_valid.predict_steps);
        x(:,:,k+1) = system.changeCoordinate(q_,param_valid,xd);
        F = [];
        if param_valid.visual_capture
            for k = 1:length(param_sets)
                plot(x(1,:,k), x(3,:,k), 'b');
                hold on
            end
            axis ij
            plot(x(1,:,k+1), x(3,:,k+1), 'r');
            hold off
            ylim([-1,7])
            xlim([-4,4])
            if param_valid.consider_collision
                for l = 1:size(param_valid.obs_pos,2)
                    objpos(l,:) = [(param_valid.obs_pos(:,l)-param_valid.obs_size(1,l)).', (2*param_valid.obs_size(:,l))*[1 1]];
                    rectangle('Position',objpos(l,:),'Curvature',[1 1]);
                end
            end
            F = getframe;
        end
    end

end

