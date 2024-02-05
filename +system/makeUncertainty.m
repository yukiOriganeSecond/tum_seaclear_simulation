function [param,W] = makeUncertainty(seed,param_base,force_deterministic)
arguments
    seed        % scalar value of random seed
    param_base  % datasets of parameter property
    force_deterministic = false
end
    rng(seed);  % reset random value generator
    param = struct;
    for fname = string(fieldnames(param_base)).'
        if (param_base.force_deterministic.average)||(force_deterministic)
            param.(fname) = param_base.(fname).uncertainCalc(true); % sample parameters
        else
            param.(fname) = param_base.(fname).uncertainCalc(false); % sample parameters
        end
    end
    dW = sqrt(param.dt)*randn(1,param.Nt);
    W = cumsum(dW); % generate Wiener Process 
end

