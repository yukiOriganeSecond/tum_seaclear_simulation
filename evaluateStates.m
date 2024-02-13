function result = evaluateStates(q,xd,param,Q,R,P)

    x = system.changeCoordinate(q,param);
    result = 0;
    result = result + param.dt*sum(dot(Q*(x(:,:)-xd(:,1)),(x(:,:)-xd(:,1))));
    if param.consider_collision
        result = result + param.constraint_penalty.* (min(vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size)<0 );
        %result = result + param.constraint_penalty.* exp(-(min(vecnorm(x([1,3],:)-param.obs_pos,2,1)-param.obs_size))^2);
    end
    if param.right_side_constraints == true
        result = result + param.constraint_penalty.* any((x(1,:)<0).*(x(3,:)>4));
    end
    result = result + 1000*(min(x(3,:)-0.5)<0);
    result = result + dot(P*(x(:,end)-xd(:,1)),(x(:,end)-xd(:,1))); %termination cost
end