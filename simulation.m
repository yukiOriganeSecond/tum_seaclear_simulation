clear
%clear system.step
clear evaluateInput
param = struct;
%% 
% simulation parameters
param.dt = 0.05;
param.Nt = 200;
t_vec = param.dt:param.dt:param.dt*param.Nt;
% state variables
q = zeros(8,param.Nt);    % theta, theta_dot, l, l_dot, X, X_dot, r, r_dot
q0 = [pi/6;0;6;0;0;0;6;0];
q(:,1) = q0;
x = zeros(4,param.Nt);    % position of a robot (x,x_dot,d,d_dot)

% targets
xd = [0; 0; 1; 0];  % target value of (x; x_dot; d; d_dot);

% set viscocity
param.mu = 400;         % viscocity of robot
param.Mu_X = 1000;         % viscocity of vessel
param.Mu_l = 300;       % viscocity of wire

% other constants
param.m = 120;                           % mass of robots (kg)
floating_mass = 30;                     % Floating mass of robots and litter (kg)
param.M = 1075;                         % mass of vessel (kg)
param.I_l = 30;                         % Inertia to change wire length (kg)
param.bar_m = param.m-floating_mass;    % mass of robot under water (substituting floating force)
param.g = 9.8;                          % gravitational acceleration (m/s^2)

% set constraints
param.use_constraint = "thruster";
%param.use_constraint = "none";
param.lb = repmat([-400; -400; -6000; -6000],1,param.Nt);
param.ub = repmat([400; 400; 6000; 6000],1,param.Nt);
% Optimize Weight Matrix
%Q = diag([1,1,1,1]);    % cost matrix for state (x, d)
Q = diag([0,0,0,0]);
R = diag([1, 1, 1, 1])./(param.m^2);      % cost matrix for input (u_theta, u_r, U_l, U_X)
W = diag([10000,10000,10000,10000]); % termination cost matrix for state (x, d)

% constant inputs
%u = zeros(3,param.Nt);    % u,U_l,U_X
%u = repmat([0;0;-300;0],param.Nt);
%u = 0;      % input for robot (N)
%U_l = 0;      % input for crane (wire control) (m/s^2)
%U_X = 1000;      % input for vessel position      (m/s^2)
%u0 = zeros(4,param.Nt);
%u0 = repmat([0;-param.bar_m*param.g;0;0],[1,param.Nt]);
u0 = repmat([0;0;-param.bar_m*param.g;0],[1,param.Nt]);
%u0 = repmat([0;-param.bar_m*param.g/2;-param.bar_m*param.g/2;0],[1,param.Nt]);
%u0 = repmat([0;0;0;0],[1,param.Nt]);
%u0 = u_b;
param.enable_u = [
    0;
    0;
    1;
    1];  % do not use u_r

%% optimization
clc
options = optimoptions(@fmincon,'MaxFunctionEvaluations',30000,'MaxFunctionEvaluations',3*10^5);
tic
for opt_cnt = size(param.enable_u,2)
    if param.use_constraint == "thruster"
        [u,fval] = fmincon(@(u)evaluateInput(u,q0,xd,Q,R,W,param,opt_cnt),u0,[],[],[],[],param.lb,param.ub,[],options);
    else
        [u,fval] = fmincon(@(u)evaluateInput(u,q0,xd,Q,R,W,param,opt_cnt),u0,[],[],[],[],[],[],[],options);
    end
    u0 = u; % repeat optimization using former solution as initial solution
end
toc
disp(fval)

%% simulation
%evaluateInput(u,q0,xd,Q,R,W,param)
%u = u0;opt_cnt = 1;
q = system.steps(q0,u,param,opt_cnt);
x = system.changeCoordinate(q,param);

%% save
folder_name = "data/"+string(datetime('now','Format','yyyyMMdd/HH_mm_ss/'));
mkdir(folder_name);
save(folder_name+"simulation.mat")
   
%% Visualize
visual.visualInit();
visual.plotInputs(u,param,t_vec,[2,3],folder_name);
visual.plotRobotStates(q,param,t_vec,[7,8],folder_name);
visual.plotRobotOutputs(x,xd,param,t_vec,[3 4],folder_name);
%visual.plotInputs(u,param,t_vec,[1,2;3,4],folder_name);
%visual.plotRobotStates(q,param,t_vec,[1,7,5;2,8,6],folder_name);
%visual.plotRobotStates(q,param,t_vec,[5;6],folder_name);
%visual.plotRobotOutputs(x,xd,param,t_vec,[1,3;2,4],folder_name);
%visual.plotAbsolutePath(q,x,param,t_vec,folder_name);
%visual.plotRelativePath(q,x,param,t_vec,folder_name);
%visual.makeSnaps(q,x,param,t_vec,folder_name,[1,40,80;120,160,200]);
%visual.makePathMovie(q,x,param,t_vec,folder_name,2);

%plot(u0(2,:))
%plot(u_b(2,:))