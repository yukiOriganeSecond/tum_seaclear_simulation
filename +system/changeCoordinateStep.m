function x = changeCoordinateStep(q,param,t)
    arguments
        q
        param
        t
    end
    x = zeros(4,1);
    theta = q(1,t);
    theta_dot= q(2,t);
    l = q(3,t);
    l_dot = q(4,t);
    X = q(5,t);
    X_dot = q(6,t);
    r = q(7,t);
    r_dot = q(8,t);
    x(2,1) = -r_dot*sin(theta)-r*theta_dot*cos(theta)+X_dot;
    x(4,1) = r_dot*cos(theta)-r*theta_dot*sin(theta);
    x(1,1) = -r*sin(theta)+X;
    x(3,1) = r*cos(theta);
end