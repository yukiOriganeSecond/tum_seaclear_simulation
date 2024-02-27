
function param_base = makeStandardParameters(method)

    arguments
        method {mustBeMember(method,["RA-SAA","RA-SAA-PID","MPPI","PID-CBF"])} 
    end
    
    param_base = struct;
    
    %% simulation parameters
    dt = 0.05;
    Nt = 200;
    param_base = system.addParam(param_base,"dt",dt,"Deterministic");
    param_base = system.addParam(param_base,"Nt",Nt,"Deterministic");
    
    %% system parameters
    % set viscocity
    param_base = system.addParam(param_base,"mu_r",[120 0 0],"White",0.20);   % viscocity of robot
    param_base = system.addParam(param_base,"mu_theta",[120 0 0],"White",0.20);   % viscocity of robot
    param_base = system.addParam(param_base,"Mu_X",[0 1000 0],"Deterministic",0.30);   % viscocity of vessel
    param_base = system.addParam(param_base,"Mu_l",[0 300 0],"Deterministic",0.30);   % viscocity of wire
    
    % other constants
    param_base = system.addParam(param_base,"m",120,"White",0.20);       % mass of robots (kg)
    param_base = system.addParam(param_base,"M",1075,"Deterministic",0.01);      % mass of vessel (kg)
    param_base = system.addParam(param_base,"I_l",30,"Deterministic",0.10);      % Inertia to change wire length (kg)
    param_base = system.addParam(param_base,"bar_m",90,"White",0.20);   % mass of robot under water (substituting floating force)
    param_base = system.addParam(param_base,"g",9.8,"Deterministic");            % gravitational acceleration (m/s^2)
    
    % set time delay of input. if set as dt, it is same as non delay
    param_base = system.addParam(param_base,"T",[0.1; 0.1; 0.5; 1.0],"Deterministic");  % T_theta; T_r; T_l; T_X 
    
    % set limitations
    lb = [-400; -400; -6000; -6000];
    ub = [400; 400; 6000; 6000];
    param_base = system.addParam(param_base,"lb",lb(:,1),"Deterministic",0);
    param_base = system.addParam(param_base,"ub",ub(:,1),"Deterministic",0);
    
    % set noise
    param_base = system.addParam(param_base,"W_effect",[0; 0; 0; 0],"Deterministic");   % set wiener effect
    param_base = system.addParam(param_base,"sensing_noise",0.2*[1; 1; 1; 1; 1; 1; 1; 1],"Deterministic");
    
    %% Controller parameter
    param_base = system.addParam(param_base,"low_side_controller","PID","Deterministic");
    param_base = system.addParam(param_base,"kp",[800;800;800;800],"Deterministic");
    param_base = system.addParam(param_base,"ki",[0;0;0;0],"Deterministic");
    param_base = system.addParam(param_base,"kd",[80;80;80;80],"Deterministic");
    
    %% problem setting
    
    % initial values
    % state variables : q = [theta, theta_dot, l, l_dot, X, X_dot, r, r_dot]
    % output variables: x = (x,x_dot,d,d_dot)
    param_base = system.addParam(param_base,"q0",[pi/6;0;6;0;0;0;6;0],"White",[0;0;0;0;0;0;0;0]);
    
    % targets
    %xd = [0; 0; 1; 0];  % target value of (x; x_dot; d; d_dot);
    xd = [0; 0; 1; 0; 0; 0];    % target value of (x; x_dot; d; d_dot; X; X_dot);
    param_base = system.addParam(param_base,"xd",[0; 0; 1; 0; 0; 0],"Deterministic");
    param_base = system.addParam(param_base,"equality_slack",[0.3; 0.3],"Deterministic");   % slack variables for termination constraint [x; xdot]
    
    % set input rate
    input_prescale = 1; % standard common prescale is 1
    param_base = system.addParam(param_base,"input_prescale",input_prescale,"Deterministic");
    
    % set constraints
    param_base = system.addParam(param_base,"obs_pos",[[0;4.5],[0;6]],"Deterministic",[0.10 0.10 0.10]);
    param_base = system.addParam(param_base,"obs_size",[1 1],"Deterministic",0.1);
    param_base = system.addParam(param_base,"ground_depth",20,"Deterministic");
    param_base = system.addParam(param_base,"right_side",10,"Deterministic");
    param_base = system.addParam(param_base,"alpha",0.02,"Deterministic");
    param_base = system.addParam(param_base,"t",-0.2,"Deterministic");
    %param_base = system.addParam(param_base,"consider_collision",false,"Deterministic");    % if false, obstacles is ignored
    param_base = system.addParam(param_base,"consider_collision",true,"Deterministic");    % if false, obstacles is ignored
    param_base = system.addParam(param_base,"right_side_constraints",true,"Deterministic");
    
    % Optimize Weight Matrix
    param_base = system.addParam(param_base,"Q",zeros(length(xd)),"Deterministic");   % cost matrix for state (x, d)
    param_base = system.addParam(param_base,"R",diag([1, 1, 1, 1])./(param_base.m.average^2),"Deterministic");   % cost matrix for input (u_theta, u_r, U_l, U_X)  
    param_base = system.addParam(param_base,"P",zeros(length(xd)),"Deterministic");
    gravity_force = param_base.bar_m.average*param_base.g.average;
    param_base = system.addParam(param_base,"u0",[0;-gravity_force/2;-gravity_force/2;0],"Deterministic");  % TODO: repmat
    param_base = system.addParam(param_base,"f0",[0; 0; -gravity_force; 0],"Deterministic");    % initial value of force input theta,r,l,X
    
    %% define method depended parameters
    if ismember(method,["RA-SAA","RA-SAA-PID"])
        input_prescale = 8; % change input prescale
        param_base = system.addParam(param_base,"input_prescale",input_prescale,"Deterministic");
        if method == "RA-SAA"
            param_base = system.addParam(param_base,"low_side_controller","none","Deterministic");
        else    % RA-SAA-PID 
            param_base = system.addParam(param_base,"low_side_controller","PID","Deterministic");
        end
    elseif method == "MPPI"
    elseif method == "PID-CBF"
        % controller
        param_base = system.addParam(param_base,"low_side_controller","PID","Deterministic");
        param_base = system.addParam(param_base,"use_gravity_compensate",true,"Deterministic");
        % target
        param_base = system.addParam(param_base,"use_heuristic_trajectory",true,"Deterministic");
        %param_base = system.addParam(param_base,"qd",[0;0;1;0;0;0;1;0],"Deterministic");    % target state
        % noise
        param_base = system.addParam(param_base,"acc_noise",0.2*[1; 1; 1; 1],"Deterministic");
        param_base = system.addParam(param_base,"force_noise_coeff",0.2*[1; 1; 1; 1],"Deterministic");
        param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");
        % CBF
        param_base = system.addParam(param_base,"enable_CBF",true,"Deterministic"); % if false, no cbf
        %param_base = system.addParam(param_base,"gamma",[2 3 0.1],"Deterministic");
        param_base = system.addParam(param_base,"gamma",[10 10 1],"Deterministic");
        % Optimize Weight Matrix
        param_base = system.addParam(param_base,"Q",diag([100,100,100,100]),"Deterministic");   % cost matrix for state (x, d)
        param_base = system.addParam(param_base,"R",diag([1, 1, 1, 1])./(param_base.m.average^2),"Deterministic");   % cost matrix for input (u_theta, u_r, U_l, U_X)  
        param_base = system.addParam(param_base,"P",diag([10000,10000,10000,10000]),"Deterministic");
        % initial solution
        gravity_force = param_base.bar_m.average*param_base.g.average;
        param_base = system.addParam(param_base,"u0",[0; 0;-gravity_force/2;0],"Deterministic");
        param_base = system.addParam(param_base,"f0",[0; 0; -gravity_force; 0],"Deterministic");    % initial value of force input theta,r,l,X
    end
end
