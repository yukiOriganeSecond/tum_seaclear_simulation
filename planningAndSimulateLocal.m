function [q,f,u,param_nominal,param_sim,find_feasible_solution] = planningAndSimulateLocal(param_base,seed_list)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述
    
    [param_nominal,W_nominal] = system.makeUncertainty(1, param_base, true);
    Nt = param_nominal.Nt;
    Ns = length(seed_list);
    q = zeros(length(param_nominal.q0),Nt,Ns); % state variables
    f = zeros(length(param_nominal.u0(:,1)),Nt,Ns);  % force input
    u = zeros(length(param_nominal.u0(:,1)),Nt,Ns);
    u0 = repmat(param_nominal.u0,1,param_nominal.Nt);
    face_infeasible = zeros(Ns,1);
    cbf = system.CBF;
    
    i = 0;
    for seed = seed_list
        i = i+1;
        disp("(Local) processing sample "+string(i))
        [param_sim(i), W] = system.makeUncertainty(seed, param_base, false);
        q(:,1,i) = param_sim(i).q0;
        f(:,1,i) = param_sim(i).f0;
        [q(:,:,i), f(:,:,i), u(:,:,i), face_infeasible(i,1)] = system.stepsFBwithCBF(param_sim(i).q0,param_sim(i),param_nominal,W,cbf);
    end
    find_feasible_solution = mean(~face_infeasible);
    %rng(seed_valid);
end

