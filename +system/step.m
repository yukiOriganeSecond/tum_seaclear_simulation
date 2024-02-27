
function [q_next, f_next, mode, q_ddot] = step(qt, ft, ut, param, mode, dW)
    arguments
        qt      % state vector at time t
        ft      % force input vector at time t
        ut      % signal input vector at time t
        param   % parameter sets
        mode    % hybrid mode
        dW      % time difference of Winner process
    end
    theta = qt(1);
    theta_dot= qt(2);
    l = qt(3);
    l_dot = qt(4);
    X = qt(5);
    X_dot = qt(6);
    r = qt(7);
    r_dot = qt(8);
    f_next = ft + param.dt./param.T.*(-ft+ut);
    f_theta = ft(1);
    f_r = ft(2);
    F_l = ft(3);
    F_X = ft(4);
    
    q_ddot = zeros(4,1);    % theta, l, X, r

    if (r<0.01) || abs(theta)>pi
        q_next = qt;
        return;
    end
    
    ang_vel_vs_water = r*theta_dot-X_dot*cos(theta);
    drag_force_theta = -param.mu_theta(1)*(abs(ang_vel_vs_water)*ang_vel_vs_water)-param.mu_theta(2)*(ang_vel_vs_water)-param.mu_theta(3)*param.bar_m*sign(ang_vel_vs_water)*cos(theta);
    drag_force_r = -param.mu_r(1)*abs(r_dot)*r_dot-param.mu_r(2)*r_dot-param.mu_r(3)*param.bar_m*sign(r_dot)*sin(theta);
    drag_force_l = -param.Mu_l(1)*abs(l_dot)*l_dot-param.Mu_l(2)*l_dot-param.Mu_l(3)*sign(l_dot);
    drag_force_X = -param.Mu_X(1)*abs(X_dot)*X_dot-param.Mu_X(2)*X_dot-param.Mu_X(3)*sign(X_dot);
    
    q_ddot(3) = (F_X+drag_force_X)/param.M;
    %theta_ddot = (u+param.m*X_ddot*cos(theta)-param.bar_m*param.g*sin(theta)-param.mu*(l*theta_dot-X_dot*cos(theta))-2*l_dot*theta_dot)/l/param.m;
    q_ddot(1) = (f_theta+param.m*q_ddot(3)*cos(theta)-param.bar_m*param.g*sin(theta)-2*param.m*r_dot*theta_dot+drag_force_theta)/r/param.m;
    %q_ddot(1) = (f_theta+param.m*q_ddot(3)*cos(theta)-param.bar_m*param.g*sin(theta)+drag_force_theta)/r/param.m;
    

    %if r>=l
    T = 0;  % tention force
    if mode == 1    % with wire tention
        q_ddot(4) = (param.m*q_ddot(3)*sin(theta)+param.m*r*theta_dot^2+param.bar_m*param.g*cos(theta)+drag_force_r+drag_force_l+f_r+F_l)/(param.m+param.I_l);
        q_ddot(2) = q_ddot(4);
        T = param.I_l*q_ddot(2) - drag_force_l-F_l;
    else            % zero wire tention
        q_ddot(4) = (param.m*q_ddot(3)*sin(theta)+param.m*r*theta_dot^2+param.bar_m*param.g*cos(theta)+drag_force_r+f_r)/param.m;
        q_ddot(2) = (F_l+drag_force_l)/param.I_l;
    end
    %q_ddot([2,4]) = 0;  % DEBUG CODE HERE !!!!
    q_next = zeros(8,1);
    q_next([2,4,6,8],1) = qt([2,4,6,8],1) + param.dt*q_ddot;
    q_next([1,3,5,7],1) = qt([1,3,5,7],1) + param.dt*qt([2,4,6,8],1);
    %theta_dot_next = theta_dot + param.dt * theta_ddot;
    %l_dot_next = l_dot + param.dt*l_ddot;
    %X_dot_next = X_dot + param.dt*X_ddot;
    %r_dot_next = r_dot + param.dt*r_ddot;
    
    %theta_next = theta + param.dt*theta_dot;
    %l_next = l + param.dt*l_dot;
    %X_next = X + param.dt*X_dot;
    %r_next = r + param.dt*r_dot;

    if (mode == 1)&&(T<=0)
        mode = 2;   % swich to zero wire tention mode
    elseif (mode == 2)&&(q_next(7)>=q_next(3))&&(q_next(8)>q_next(4))
        mode = 1;   % swich to with wire tention mode
        q_next(8) = (param.m*q_next(8)+param.I_l*q_next(4))/(param.m+param.I_l);
        q_next(4) = q_next(8);  % conservation of momentum
    end

    if anynan(q_next)
        q_next = qt;
    end

    %q_next(2) = theta_dot_next;
    %q_next(4) = l_dot_next;
    %q_next(6) = X_dot_next;
    %q_next(8) = r_dot_next;
    
    %q_next(1) = theta_next;
    %q_next(3) = l_next;
    %q_next(5) = X_next;
    %q_next(7) = r_next;

    q_next([2,4,6,8]) = q_next([2,4,6,8]) + dW*param.W_effect;
end