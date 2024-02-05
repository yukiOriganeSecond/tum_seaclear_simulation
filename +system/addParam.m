function param_base = addParam(param_base,name,average,type,error)
arguments
    param_base
    name
    average     % base value or vector, Matrix
    type {mustBeMember(type,{'Deterministic','Gaussian','White'})} = 'Deterministic';
    error = 0   % error ratio. ex) \pm 0.03 = \pm 30%
end
    param_base.(name) = system.ParamBaseElement;
    param_base.(name).type = type;
    param_base.(name).average = average;
    param_base.(name).error = error;
end

