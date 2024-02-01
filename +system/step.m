
function [q_next, f_next, mode] = step(qt, ft, ut, param, mode, opt_cnt, dW)
    arguments
        qt      % state vector at time t
        ft      % force input vector at time t
        ut      % signal input vector at time t
        param   % parameter sets
        mode    % hybrid mode
        opt_cnt % optimization count
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
    %u_theta = ut(1)*param.enable_u(1,opt_cnt);
    %u_r = ut(2)*param.enable_u(2,opt_cnt);
    %U_l = ut(3)*param.enable_u(3,opt_cnt);
    %U_X = ut(4)*param.enable_u(4,opt_cnt);
    f_next = ft + param.dt./param.T.*(-ft+ut.*param.enable_u);
    f_theta = ft(1);
    f_r = ft(2);
    F_l = ft(3);
    F_X = ft(4);
    
    X_ddot = (F_X-X_dot*param.Mu_X)/param.M;
    %theta_ddot = (u+param.m*X_ddot*cos(theta)-param.bar_m*param.g*sin(theta)-param.mu*(l*theta_dot-X_dot*cos(theta))-2*l_dot*theta_dot)/l/param.m;
    theta_ddot = (f_theta+param.m*X_ddot*cos(theta)-param.bar_m*param.g*sin(theta)-param.mu*(r*theta_dot-X_dot*cos(theta)))/r/param.m;
    
    %if r>=l
    T = 0;  % tention force
    if mode == 1    % with wire tention
        r_ddot = (param.m*X_ddot*sin(theta)+param.m*r*theta_dot^2+param.bar_m*param.g*cos(theta)-(param.mu+param.Mu_l)*r_dot+f_r+F_l)/(param.m+param.I_l);
        l_ddot = r_ddot;
        T = param.I_l*l_ddot + param.Mu_l*l_dot-F_l;
    else            % zero wire tention
        r_ddot = (param.m*X_ddot*sin(theta)+param.m*r*theta_dot^2+param.bar_m*param.g*cos(theta)-param.mu*r_dot+f_r)/param.m;
        l_ddot = (F_l-l_dot*param.Mu_l)/param.I_l;
    end
    theta_dot_next = theta_dot + param.dt * theta_ddot;
    l_dot_next = l_dot + param.dt*l_ddot;
    X_dot_next = X_dot + param.dt*X_ddot;
    r_dot_next = r_dot + param.dt*r_ddot;
    
    theta_next = theta + param.dt*theta_dot;
    l_next = l + param.dt*l_dot;
    X_next = X + param.dt*X_dot;
    r_next = r + param.dt*r_dot;

    if (mode == 1)&&(T<=0)
        mode = 2;   % swich to zero wire tention mode
    elseif (mode == 2)&&(r_next>=l_next)&&(r_dot_next>l_dot_next)
        mode = 1;   % swich to with wire tention mode
        r_dot_next = (param.m*r_dot_next+param.I_l*l_dot_next)/(param.m+param.I_l);
        l_dot_next = r_dot_next;  % conservation of momentum
    end

    q_next(2) = theta_dot_next;
    q_next(4) = l_dot_next;
    q_next(6) = X_dot_next;
    q_next(8) = r_dot_next;
    
    q_next(1) = theta_next;
    q_next(3) = l_next;
    q_next(5) = X_next;
    q_next(7) = r_next;
end