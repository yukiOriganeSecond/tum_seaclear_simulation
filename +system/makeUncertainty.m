function [param,W] = makeUncertainty(seed,param_base)
arguments
    seed        % scalar value of random seed
    param_base  % datasets of parameter property
end
    rng(seed);  % reset random value generator
    param = struct;
    for fname = string(fieldnames(param_base)).'
        if (param_base.force_deterministic.average)
            param_base.(fname).type = 'Deterministic';  % force deterministic
        end
        param.(fname) = param_base.(fname).uncertainCalc(); % sample parameters
    end
    dW = sqrt(param.dt)*randn(1,param.Nt);
    W = cumsum(dW); % generate Wiener Process 
end

