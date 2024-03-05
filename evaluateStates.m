function [result,violate_constraint] = evaluateStates(q,xd,param)
    violate_constraint = false;
    x = system.changeCoordinate(q,param,xd);
    result = 0;
    result = result + param.dt*sum(dot(param.Q*(x(:,:)-xd(:,1)),(x(:,:)-xd(:,1))));
    if param.consider_collision
        dist_ = zeros(size(param.obs_pos,2),1);
        for j = 1:size(param.obs_pos,2)
            dist_(j) = min(vecnorm(x([1,3],:)-param.obs_pos(:,j),2,1)-param.obs_size(:,j));
        end
        if min(dist_)<0
            result = result + param.constraint_penalty;
            violate_constraint = true;
        end
        %result = result + param.constraint_penalty.* exp(-(min(vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size))^2);
    end
    if param.right_side_constraints == true
        result = result + 2*param.constraint_penalty.* any((x(1,:)>0).*(x(3,:)>4));
    end
    %result = result + param.constraint_penalty*any((x(3,:)>5.5));
    result = result + param.constraint_penalty*any((x(3,:)<0));
    result = result + dot(param.P*(x(:,end)-xd(:,1)),(x(:,end)-xd(:,1))); %termination cost
end