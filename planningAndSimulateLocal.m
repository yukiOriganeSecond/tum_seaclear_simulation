function [q,f,u,param_nominal] = planningAndSimulateLocal(u0,xd,param_base,seed_list,lb,ub)
%UNTITLED この関数の概要をここに記述
%   詳細説明をここに記述
    
    [param_nominal,W_nominal] = system.makeUncertainty(1, param_base, true);
    Nt = param_nominal.Nt;
    Ns = length(seed_list);
    q = zeros(length(param_nominal.q0),Nt,Ns); % state variables
    f = zeros(length(u0(:,1)),Nt,Ns);  % force input
    u = zeros(length(u0(:,1)),Nt,Ns);
    cbf = system.CBF;

    i = 0;
    for seed = seed_list
        i = i+1;
        [param, W] = system.makeUncertainty(seed, param_base, false);
        q(:,1,i) = param.q0;
        f(:,1,i) = param.f0;
        [q(:,:,i), f(:,:,i), u(:,:,i)] = system.stepsFBwithCBF(param.q0,param,param_nominal,W,cbf);
    end
    %rng(seed_valid);
end

