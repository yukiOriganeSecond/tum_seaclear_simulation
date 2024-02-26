clear
clc
%clear system.step
clear ControllerPID
clear evaluateInput
param_base = struct;
% parpool
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
%xd = [0; 0; 1; 0];  % target value of (x; x_dot; d; d_dot);
xd = [0; 0; 1; 0; 0; 0];    % target value of (x; x_dot; d; d_dot; X; X_dot);
param_base = system.addParam(param_base,"xd",xd,"Deterministic");
%xd = [0; 0; 1; 0];  % target value of (theta; theta_dot; r; r_dot);
param_base = system.addParam(param_base,"equality_slack",[0.3; 0.3],"Deterministic");   % slack variables for termination constraint [x; xdot]

% set input rate
input_prescale = 8;
Nu = Nt/input_prescale;
param_base = system.addParam(param_base,"input_prescale",input_prescale,"Deterministic");

% set time delay of input. if set as dt, it is same as non delay
param_base = system.addParam(param_base,"T",[0.1; 0.1; 0.5; 1.0],"Deterministic");  % T_theta; T_r; T_l; T_X 
%param_base = system.addParam(param_base,"T",[0.05; 0.05; 0.05; 0.05],"Deterministic");  % T_theta; T_r; T_l; T_X 

% set noise
%param_base = system.addParam(param_base,"W_effect",[0.1; 0.1; 0.1; 0.1],"Deterministic");
param_base = system.addParam(param_base,"W_effect",[0; 0; 0; 0],"Deterministic");   % set wiener effect
param_base = system.addParam(param_base,"sensing_noise",0.2*[1; 1; 1; 1; 1; 1; 1; 1],"Deterministic");
%param_base = system.addParam(param_base,"sensing_noise",[0; 0; 0; 0],"Deterministic");

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
param_base = system.addParam(param_base,"obs_pos",[[0;4.5],[0;6]],"Deterministic",[0.10 0.10 0.10]);
param_base = system.addParam(param_base,"obs_size",[1 1],"Deterministic",0.1);
param_base = system.addParam(param_base,"ground_depth",20,"Deterministic");
param_base = system.addParam(param_base,"right_side",0,"Deterministic");
param_base = system.addParam(param_base,"alpha",0.02,"Deterministic");
param_base = system.addParam(param_base,"t",-0.2,"Deterministic");
%param_base = system.addParam(param_base,"consider_collision",false,"Deterministic");    % if false, obstacles is ignored
param_base = system.addParam(param_base,"consider_collision",true,"Deterministic");    % if false, obstacles is ignored
param_base = system.addParam(param_base,"right_side_constraints",true,"Deterministic");

% set limitations
use_constraint = "thruster";
%param.use_constraint = "none";
lb = [-400; -400; -6000; -6000];
ub = [400; 400; 6000; 6000];
param_base = system.addParam(param_base,"lb",lb(:,1),"Deterministic",0);
param_base = system.addParam(param_base,"ub",ub(:,1),"Deterministic",0);
% Optimize Weight Matrix
%Q = diag([1,1,1,1]);    % cost matrix for state (x, d)
param_base = system.addParam(param_base,"Q",zeros(length(xd)),"Deterministic");   % cost matrix for state (x, d)
param_base = system.addParam(param_base,"R",diag([1, 1, 1, 1])./(param_base.m.average^2),"Deterministic");   % cost matrix for input (u_theta, u_r, U_l, U_X)  
param_base = system.addParam(param_base,"P",zeros(length(xd)),"Deterministic");
% Set Low side controller
%param_base = system.addParam(param_base,"low_side_controller","none","Deterministic");
param_base = system.addParam(param_base,"low_side_controller","PID","Deterministic");
param_base = system.addParam(param_base,"kp",[800;800;800;800],"Deterministic");
param_base = system.addParam(param_base,"ki",[0;0;0;0],"Deterministic");
param_base = system.addParam(param_base,"kd",[80;80;80;80],"Deterministic");

% constant inputs
%u0 = repmat([0;0;-param_base.bar_m.average*param_base.g.average;0],[1,Nt]);
%u0 = repmat([0;-param_base.bar_m.average*param_base.g.average/2;-param_base.bar_m.average*param_base.g.average/2;0],[1,Nt]);
gravity_force = param_base.bar_m.average*param_base.g.average;
param_base = system.addParam(param_base,"u0",[0;-gravity_force/2;-gravity_force/2;0],"Deterministic");  % TODO: repmat
param_base = system.addParam(param_base,"f0",[0; 0; -param_base.bar_m.average*param_base.g.average; 0],"Deterministic");    % initial value of force input theta,r,l,X

%u0 = repmat([0;0;0;0],[1,param.Nu]);
%u0 = u_b;

seed_plan = 1;
seed_simulate = 1:20;
[q,f,u,param_nominal] = planningAndSimulateSAA(param_base,seed_plan,seed_simulate);
for i = 1:length(seed_simulate)
    x(:,:,i) = system.changeCoordinate(q(:,:,i),param_nominal);
end
max_energy_consumption = energyEvaluation(u(:,:,:),f(:,:,:),param_nominal);

%% save
folder_name = "data/"+string(datetime('now','Format','yyyyMMdd/HH_mm_ss/'));
mkdir(folder_name);
save(folder_name+"simulation.mat")

%% Visualize
t_vec = dt:dt:dt*Nt;
snum_list = 1:length(seed_simulate);
%snum_list = [1];
visual.visualInit();
%visual.plotInputs(u,f,param_nominal,t_vec,[2,3],folder_name);
visual.plotRobotStates(q,param_nominal,t_vec,[7,8],folder_name,snum_list);
visual.plotRobotOutputs(x,xd,param_nominal,t_vec,[1 3; 2 4],folder_name,snum_list);
visual.plotInputs(u,f,param_nominal,t_vec,[1,2;3,4],folder_name,snum_list);
%visual.plotInputsFB(u,u_val,f,param_nominal,t_vec,[1,2;3,4],folder_name,snum_list);
%visual.plotRobotOutputsFB(x,xd,x_nominal,x_nonFB,param_nominal,t_vec,[1,3;2,4],folder_name,snum_list);
%visual.plotRobotStatesFB(q,q_nominal,q_nonFB,param_nominal,t_vec,[1,7;2,8],folder_name,1:length(seed_list));
visual.plotRobotStatesFB(q,q_nominal,q_nonFB,param_nominal,t_vec,[1;2],folder_name,1:length(seed_list));
%visual.plotRobotStatesErrorFB(q,q_nominal,q_nonFB,param_nominal,t_vec,[1,7;2,8],folder_name,1:length(seed_list));
%visual.plotRobotStates(q,param_nominal,t_vec,[1,7,5;2,8,6],folder_name,1:length(seed_list));
%visual.plotRobotStates(q,param_nominal,t_vec,[5;6],folder_name);
%visual.plotRobotOutputs(x,xd,param_nominal,t_vec,[1,3;2,4],folder_name);
%visual.plotAbsolutePath(q,x,param_nominal,t_vec,folder_name);
%visual.plotRelativePath(q,x,param_nominal,t_vec,folder_name);
%visual.makeSnaps(q,x,param_nominal,t_vec,folder_name,[1,40,80;120,160,200],snum_list);
visual.makeSnaps(q,x,param_nominal,t_vec,folder_name,[1],snum_list);
visual.makeSnapsFB(q,q_nonFB,q_nominal,x,x_nonFB,x_nominal,param,t_vec,folder_name,[1],snum_list);
%title("\alpha = "+string(param_nominal.alpha)+", val = "+string(fval))
title("\alpha = "+string(param_nominal.alpha)+", max energy consumption = "+string(max_energy_consumption))
%visual.makePathMovie(q,x,param_nominal,t_vec,folder_name,1,snum_list);

%plot(u0(2,:))
%plot(u_b(2,:))