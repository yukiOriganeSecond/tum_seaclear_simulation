function [q,f,u,param_nominal,param_sim_list,find_feasible_solution,F_] = planningAndSimulateMPPI(param_base,seed_plan_list,seed_simulate_list)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述

    [param_nominal,~] = system.makeUncertainty(1, param_base, true);    % nominal parameters
    i = 0;
    for seed_plan = seed_plan_list
        i = i+1;
        [param_plan_list(i), W_plan_list(i,:)] = system.makeUncertainty(seed_plan, param_base, false);
    end
    i = 0;
    for seed_simulate = seed_simulate_list
        i = i+1;
        [param_sim_list(i), W_sim_list(i,:)] = system.makeUncertainty(seed_simulate, param_base, false);
    end
    %rng(seed_simulate_list);
    Nt = param_nominal.Nt;
    Nss = length(seed_simulate_list);            % number of seed simulate
    q = zeros(length(param_nominal.q0),Nt,Nss);  % state variables
    f = zeros(length(param_nominal.u0),Nt,Nss);  % force input
    u = zeros(length(param_nominal.u0),Nt,Nss);  % control input
    face_infeasible = zeros(Nss,1);
    fig = figure;

    %parfor i = 1:length(seed_simulate_list)
    for i = 1:length(seed_simulate_list)
        disp("(Local) processing sample "+string(i))
        [q(:,:,i), f(:,:,i), u(:,:,i), face_infeasible(i,1),F_] = system.stepsMPPI(param_sim_list(i),param_nominal,param_plan_list,W_sim_list(i,:),W_plan_list,fig);
    end
    find_feasible_solution = mean(~face_infeasible);
end

