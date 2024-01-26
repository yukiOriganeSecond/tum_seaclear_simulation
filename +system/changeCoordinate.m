function x = changeCoordinate(q,param)

    x = zeros(4,param.Nt);
    for t = 1:param.Nt
        x(:,t) = system.changeCoordinateStep(q,param,t);
    end
end