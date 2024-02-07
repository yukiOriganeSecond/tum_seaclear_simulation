clear
clc
%clear system.step
clear ControllerPID
clear evaluateInput
param_base = struct;
%% 
% simulation parameters
dt = 0.05;
Nt = 200;
param_base = system.addParam(param_base,"dt",dt,"Deterministic");
param_base = system.addParam(param_base,"Nt",Nt,"Deterministic");

%param_base = system.addParam(param_base,"force_deterministic",true,"Deterministic");   % if true, there is no uncertainty
%param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");

% initial values
% state variables : q = [theta, theta_dot, l, l_dot, X, X_dot, r, r_dot]
% output variables: x = (x,x_dot,d,d_dot)
param_base = system.addParam(param_base,"q0",[pi/6;0;6;0;0;0;6;0],"White",[0;0;0;0;0;0;0;0]);

% targets
xd = [0; 0; 1; 0];  % target value of (x; x_dot; d; d_dot);
%xd = [0; 0; 1; 0];  % target value of (theta; theta_dot; r; r_dot);

% set time delay of input. if set as dt, it is same as non delay
param_base = system.addParam(param_base,"T",[0.1; 0.1; 0.5; 1.0],"Deterministic");  % T_theta; T_r; T_l; T_X 
%param_base = system.addParam(param_base,"T",[0.05; 0.05; 0.05; 0.05],"Deterministic");  % T_theta; T_r; T_l; T_X 

% set noise
%param_base = system.addParam(param_base,"W_effect",[0.1; 0.1; 0.1; 0.1],"Deterministic");
param_base = system.addParam(param_base,"W_effect",[0; 0; 0; 0],"Deterministic");   % set wiener effect
%param_base = system.addParam(param_base,"sensing_noise",[0.1; 0.1; 0.1; 0.1],"Deterministic");
param_base = system.addParam(param_base,"sensing_noise",[0; 0; 0; 0],"Deterministic");

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

% set constraints
param_base = system.addParam(param_base,"obs_pos",[0;4],"Deterministic",[0.10;0.10]);
param_base = system.addParam(param_base,"obs_size",1,"Deterministic",0.1);
param_base = system.addParam(param_base,"ground_depth",5.5,"Deterministic");
param_base = system.addParam(param_base,"consider_collision",false,"Deterministic");    % if false, obstacles is ignored
%param_base = system.addParam(param_base,"consider_collision",true,"Deterministic");    % if false, obstacles is ignored

% set limitations
use_constraint = "thruster";
%param.use_constraint = "none";
lb = repmat([-400; -400; -6000; -6000],1,Nt);
ub = repmat([400; 400; 6000; 6000],1,Nt);
param_base = system.addParam(param_base,"lb",lb(:,1),"Deterministic",0);
param_base = system.addParam(param_base,"ub",ub(:,1),"Deterministic",0);
% Optimize Weight Matrix
%Q = diag([1,1,1,1]);    % cost matrix for state (x, d)
Q = diag([0,0,0,0]);
R = diag([1, 1, 1, 1])./(param_base.m.average^2);      % cost matrix for input (u_theta, u_r, U_l, U_X)
P = diag([1000,1000,1000,1000]); % termination cost matrix for state (x, d)

% Set Low side controller
%param_base = system.addParam(param_base,"low_side_controller","none","Deterministic");
param_base = system.addParam(param_base,"low_side_controller","PID","Deterministic");
param_base = system.addParam(param_base,"kp",[1000;1000;1000;1],"Deterministic");
param_base = system.addParam(param_base,"ki",[0;0;0;0],"Deterministic");
param_base = system.addParam(param_base,"kd",[0;0;0;0],"Deterministic");

% constant inputs
%u = zeros(3,param.Nt);    % u,U_l,U_X
%u = repmat([0;0;-300;0],param.Nt);
%u = 0;      % input for robot (N)
%U_l = 0;      % input for crane (wire control) (m/s^2)
%U_X = 1000;      % input for vessel position      (m/s^2)
%u0 = zeros(4,param.Nt);
%u0 = repmat([0;-param.bar_m*param.g;0;0],[1,param.Nt]);
%u0 = repmat([0;0;-param.bar_m*param.g;0],[1,param.Nt]);
u0 = repmat([0;0;-param_base.bar_m.average*param_base.g.average;0],[1,param_base.Nt.average]);
param_base = system.addParam(param_base,"f0",[0; 0; -param_base.bar_m.average*param_base.g.average; 0],"Deterministic");    % initial value of force input theta,r,l,X

%u0 = repmat([0;0;0;0],[1,param.Nt]);
%u0 = u_b;
enable_u = [
    1;
    1;
    1;
    1];  % do not use u_r at first optimization
param_base = system.addParam(param_base,"enable_u",enable_u);

%% optimization
clc
options = optimoptions(@fmincon,'MaxFunctionEvaluations',30000,'PlotFcn','optimplotfvalconstr','Display','iter');
tic
%for opt_cnt = size(param.enable_u,2)
%    if param.use_constraint == "thruster"
%        [u,fval] = fmincon(@(u)evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list),u0,[],[],[],[],lb,ub,[],options);
%    else
%        [u,fval] = fmincon(@(u)evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list),u0,[],[],[],[],[],[],[],options);
%    end
%    u0 = u; % repeat optimization using former solution as initial solution
%end
opt_cnt = 1;
seed_list = [1];
param_base = system.addParam(param_base,"force_deterministic",true,"Deterministic");
%param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");
[u,fval] = fmincon(@(u)evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list),u0,[],[],[],[],enable_u.*lb,enable_u.*ub,[],options);
%[u,fval] = fmincon(@(u)evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list),u0,[],[],[],[],enable_u.*lb,enable_u.*ub,@(u)uncertaintyConstraint(u,xd,Q,R,P,param_base,opt_cnt,seed_list),options);

u0 = u;
toc
tic
param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");
%seed_list = 1:10;
seed_list = 1;
%[u,fval] = fmincon(@(u)evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list),u0,[],[],[],[],enable_u.*lb,enable_u.*ub,[],options);
%[u,fval] = fmincon(@(u)evaluateInput(u,xd,Q,R,P,param_base,opt_cnt,seed_list),u0,[],[],[],[],enable_u.*lb,enable_u.*ub,@(u)uncertaintyConstraint(u,xd,Q,R,P,param_base,opt_cnt,seed_list),options);
toc
disp(fval)

%% simulation
%evaluateInput(u,q0,xd,Q,R,W,param)
if exist('u') == 0
    u = u0; opt_cnt = 1;
    seed_list = [1];
end
seed_list = 1;
%seed_list = 1;
%u_val = u;
%u_val = u0;
param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");
q = zeros(length(param_base.q0.average),Nt,length(seed_list));
q_nonFB = q;
q_nominal = q;
f = zeros(length(u(:,1)),Nt,length(seed_list));
x = zeros(length(xd),Nt,length(seed_list));
x_nonFB = x;
x_nominal = x;
u_val = zeros(length(u(:,1)),Nt,length(seed_list));
input_energy = zeros(length(seed_list),1);
constraint_results = zeros(length(seed_list),2);
i = 0;
for seed = seed_list
    i = i+1;
    
    [param_nominal,W] = system.makeUncertainty(seed, param_base, true); % calc nominal parameters
    [q_nominal(:,:,i),~,~] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W); % calc nominal values
    x_nominal(:,:,i) = system.changeCoordinate(q_nominal(:,:,i),param_nominal);
    [param,W] = system.makeUncertainty(seed,param_base);
    [q(:,:,i),f(:,:,i),~] = system.steps(param.q0,u,param,opt_cnt,W);   % nonFB case
    x(:,:,i) = system.changeCoordinate(q(:,:,i),param);
    u_val(:,:,i) = u(:,:);

    if param.low_side_controller ~= "none"
        q_nonFB(:,:,i) = q(:,:,i);
        x_nonFB(:,:,i) = x(:,:,i);
        [param_nominal,W] = system.makeUncertainty(seed, param_base, true); % calc nominal parameters
        [q_nominal(:,:,i),~,~] = system.steps(param_nominal.q0,u,param_nominal,opt_cnt,W); % calc nominal values
        x_nominal(:,:,i) = system.changeCoordinate(q_nominal(:,:,i),param);
        [param_unc,W] = system.makeUncertainty(seed, param_base, false); % calc uncertained parameters
        [q(:,:,i),~,u_val(:,:,i)] = system.stepsFB(param_unc.q0,q_nominal,u,param_unc,opt_cnt,W); % calc nominal values
        x(:,:,i) = system.changeCoordinate(q(:,:,i),param);
    end

    input_energy(i) = energyEvaluation(u_val(:,:,i),f(:,:,i),param.q0,xd,Q,R,P,param,opt_cnt);
    [constraint_results(i,:),Ceq] = uncertaintyConstraint(u_val(:,:,i),xd,Q,R,P,param_base,opt_cnt,seed);
end

%% save
folder_name = "data/"+string(datetime('now','Format','yyyyMMdd/HH_mm_ss/'));
mkdir(folder_name);
save(folder_name+"simulation.mat")

%% Visualize
t_vec = dt:dt:dt*Nt;
snum_list = 1:length(seed_list);
%snum_list = [1];
visual.visualInit();
visual.plotInputs(u,f,param,t_vec,[2,3],folder_name);
visual.plotRobotStates(q,param,t_vec,[7,8],folder_name,snum_list);
visual.plotRobotOutputs(x,xd,param,t_vec,[1 3; 2 4],folder_name,snum_list);
%visual.plotInputs(u,f,param,t_vec,[1,2;3,4],folder_name);
%visual.plotInputsFB(u,u_val,f,param,t_vec,[1,2;3,4],folder_name,snum_list);
%visual.plotRobotOutputsFB(x,xd,x_nominal,x_nonFB,param,t_vec,[1,3;2,4],folder_name,snum_list);
%visual.plotRobotStatesFB(q,q_nominal,q_nonFB,param,t_vec,[1,7;2,8],folder_name,1:length(seed_list));
%visual.plotRobotStatesErrorFB(q,q_nominal,q_nonFB,param,t_vec,[1,7;2,8],folder_name,1:length(seed_list));
%visual.plotRobotStates(q,param,t_vec,[1,7,5;2,8,6],folder_name,1:length(seed_list));
%visual.plotRobotStates(q,param,t_vec,[5;6],folder_name);
%visual.plotRobotOutputs(x,xd,param,t_vec,[1,3;2,4],folder_name);
%visual.plotAbsolutePath(q,x,param,t_vec,folder_name);
%visual.plotRelativePath(q,x,param,t_vec,folder_name);
%visual.makeSnaps(q,x,param,t_vec,folder_name,[1,40,80;120,160,200],snum_list);
visual.makeSnaps(q,x,param,t_vec,folder_name,[1],snum_list);
visual.makeSnapsFB(q,q_nonFB,q_nominal,x,x_nonFB,x_nominal,param,t_vec,folder_name,[1],snum_list);
%visual.makePathMovie(q,x,param,t_vec,folder_name,1,snum_list);

%plot(u0(2,:))
%plot(u_b(2,:))