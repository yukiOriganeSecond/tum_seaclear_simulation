function [q,f,u_use] = steps(q0,u,param,W,Nt)
    arguments
        q0      % initial state
        u       % time series of control input (signal input)
        param   % parameters set
        W       % Winner Process
        Nt = param.Nt % timestep
    end
    param.Nt = Nt;  % overrite by defined Nt value
    q = zeros(length(q0),param.Nt); % state variables
    f = zeros(length(u(:,1)),param.Nt);  % force input
    q(:,1) = q0;
    f(:,1) = param.f0;
    clear system.step
    if (q0(7,1)>q0(3,1))
        mode = 1;
    else
        mode = 2;
    end
    u_use = u;  % initialuze u_use by u

    for t = 1:param.Nt-1
        [q(:,t+1),f(:,t+1),mode,~] = system.step(q(:,t),f(:,t),u(:,t),param,mode,W(t+1)-W(t));
    end

end