function [q,f,u,param_nominal,param_sim,find_feasible_solution] = planningAndSimulateSAA(param_base,seed_plan,seed_simulate,~)
    
    options = optimoptions(@fmincon, ...
        'MaxFunctionEvaluations',1000, ...
        'PlotFcn','optimplotfvalconstr', ...
        'Display','iter', ...
        'SpecifyObjectiveGradient',true, ...
        'UseParallel',true, ...
        'EnableFeasibilityMode', false, ...
        'OptimalityTolerance',1e-3, ...
        'ScaleProblem',false, ...
        'StepTolerance',1e-12);

    param_base = system.addParam(param_base,"force_deterministic",true,"Deterministic");
    [param_nominal,W] = system.makeUncertainty(1, param_base, true); % calc nominal parameters

    % preplan
    param_base_preplan = param_base;
    param_base_preplan = system.addParam(param_base_preplan,"force_deterministic",true,"Deterministic");
    param_base_preplan = system.addParam(param_base_preplan,"consider_collision",false,"Deterministic");
    u0 = repmat(param_base_preplan.u0.average,1,param_base_preplan.Nt.average);
    xd = param_base_preplan.xd.average;
    [u_openloop,~,~] = planning(u0,xd,param_base_preplan,1,options);
    u0 = u_openloop;
    
    % main planning
    param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");
    param_base = system.addParam(param_base,"consider_collision",true,"Deterministic");
    [u_openloop,~,find_feasible_solution] = planning(u0,xd,param_base,seed_plan,options);
    [q_nominal(:,:),~,~] = system.steps(param_nominal.q0,u_openloop,param_nominal,W); % calc target trajectory

    % simulation
    q = zeros(length(param_nominal.q0),param_nominal.Nt,length(seed_simulate));
    f = zeros(length(param_nominal.f0),param_nominal.Nt,length(seed_simulate));
    u = zeros(length(param_nominal.u0),param_nominal.Nt,length(seed_simulate));
    
    i = 0;
    %param_sim(length(seed_simulate)) = struct;
    for seed = seed_simulate
        i = i+1;
        [param_sim(i),W] = system.makeUncertainty(seed,param_base);

        if param_sim(i).low_side_controller == "none"
            [q(:,:,i),f(:,:,i),~] = system.steps(param_sim(i).q0,u_openloop,param_sim(i),W);   % nonFB case
            u(:,:,i) = u_openloop(:,:);
        elseif param_sim(i).low_side_controller == "PID"
            %[param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
            [q(:,:,i),f(:,:,i),u(:,:,i)] = system.stepsFB(param_sim(i).q0,q_nominal,u_openloop,param_sim(i),W); %
        end
    end
end

