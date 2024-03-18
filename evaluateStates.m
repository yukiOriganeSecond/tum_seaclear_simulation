function [result,violate_constraint] = evaluateStates(q,xd,param_nominal,param_actual)
    violate_constraint = false;
    x = system.changeCoordinate(q,param_nominal,xd);
    result = 0;
    result = result + param_nominal.dt*sum(dot(param_nominal.Q*(x(:,:)-xd(:,1)),(x(:,:)-xd(:,1))));
    if param_nominal.consider_collision
        [est_pos, est_size] = system.estimateObstacleInformation(param_nominal,param_actual,q(:,1));
        dist_ = zeros(size(param_nominal.obs_pos,2),1);
        for j = 1:size(param_nominal.obs_pos,2)
            dist_(j) = min(vecnorm(x([1,3],:)-est_pos(:,j),2,1)-est_size(:,j));
        end
        if min(dist_)<0
            result = result + param_nominal.constraint_penalty;
            violate_constraint = true;
        end
        %result = result + param.constraint_penalty.* exp(-(min(vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size))^2);
    end
    if param_nominal.right_side_constraints == true
        result = result + 2*param_nominal.constraint_penalty.* any((x(1,:)>0).*(x(3,:)>4));
    end
    if size(xd,1)==4
        result = result + (vecnorm(x([1,3],end)-xd([1,3],1),2,1)-param_nominal.equality_slack(1)>0);
        result = result + (vecnorm(x([2,4],end)-xd([2,4],1),2,1)-param_nominal.equality_slack(2)>0);
    else
        result = result + (vecnorm(x([1,3,5],end)-xd([1,3,5],1),2,1)-param_nominal.equality_slack(1)>0);
        result = result + (vecnorm(x([2,4,6],end)-xd([2,4,6],1),2,1)-param_nominal.equality_slack(2)>0);
    end
    %result = result + param.constraint_penalty*any((x(3,:)>5.5));
    result = result + param_nominal.constraint_penalty*any((x(3,:)<0));
    result = result + dot(param_nominal.P*(x(:,end)-xd(:,1)),(x(:,end)-xd(:,1))); %termination cost
end