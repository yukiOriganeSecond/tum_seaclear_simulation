
clear
clc
%clear system.step
clear ControllerMPPI
clear evaluateInput
param_base = struct;
% parpool
%% 
% simulation parameters
dt = 0.05;
Nt = 200;
param_base = system.addParam(param_base,"dt",dt,"Deterministic");
param_base = system.addParam(param_base,"Nt",Nt,"Deterministic");

% initial values
% state variables : q = [theta, theta_dot, l, l_dot, X, X_dot, r, r_dot]
% output variables: x = (x,x_dot,d,d_dot)
param_base = system.addParam(param_base,"q0",[pi/6;0;6;0;0;0;6;0],"White",[0;0;0;0;0;0;0;0]);

% targets
xd = [0; 0; 1; 0];  % target value of (x; x_dot; d; d_dot);
%xd = [0; 0; 1; 0; 0; 0];
param_base = system.addParam(param_base,"xd",xd,"Deterministic");
% set input rate
input_prescale = 1;
Nu = Nt/input_prescale;
param_base = system.addParam(param_base,"input_prescale",input_prescale,"Deterministic");

% MPPI parameters
param_base = system.addParam(param_base,"predict_steps",200,"Deterministic");
param_base = system.addParam(param_base,"lambda",1000,"Deterministic");
param_base = system.addParam(param_base,"alpha_MPPI",0,"Deterministic");
param_base = system.addParam(param_base,"number_of_input_sample",10,"Deterministic");
param_base = system.addParam(param_base,"seed_for_input_sampling",100,"White",1.0); % from 0 to 200

% set time delay of input. if set as dt, it is same as non delay
param_base = system.addParam(param_base,"T",[0.1; 0.1; 0.5; 1.0],"Deterministic");  % T_theta; T_r; T_l; T_X 
%param_base = system.addParam(param_base,"T",[0.05; 0.05; 0.05; 0.05],"Deterministic");  % T_theta; T_r; T_l; T_X 

% set noise
%param_base = system.addParam(param_base,"W_effect",[0.1; 0.1; 0.1; 0.1],"Deterministic");
param_base = system.addParam(param_base,"W_effect",[0; 0; 0; 0],"Deterministic");   % set wiener effect
param_base = system.addParam(param_base,"sensing_noise",[0.1; 0.1; 0.1; 0.1],"Deterministic");
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
param_base = system.addParam(param_base,"constraint_penalty",1000^2,"Deterministic");
param_base = system.addParam(param_base,"obs_pos",[[0;4.5],[0;6]],"Deterministic",[0.10;0.10]);
param_base = system.addParam(param_base,"obs_size",[1 1 1],"Deterministic",0.1);
param_base = system.addParam(param_base,"ground_depth",20,"Deterministic");
param_base = system.addParam(param_base,"right_side",0,"Deterministic");
param_base = system.addParam(param_base,"alpha",0.5,"Deterministic");
param_base = system.addParam(param_base,"t",-0.2,"Deterministic");
%param_base = system.addParam(param_base,"consider_collision",false,"Deterministic");    % if false, obstacles is ignored
param_base = system.addParam(param_base,"consider_collision",true,"Deterministic");    % if false, obstacles is ignored
param_base = system.addParam(param_base,"right_side_constraints",false,"Deterministic");

% set limitations
use_constraint = "thruster";
%param.use_constraint = "none";
lb = [-400; -400; -6000; -6000];
ub = [400; 400; 6000; 6000];
param_base = system.addParam(param_base,"lb",lb(:,1),"Deterministic",0);
param_base = system.addParam(param_base,"ub",ub(:,1),"Deterministic",0);
% Optimize Weight Matrix
param_base = system.addParam(param_base,"Q",diag([100,100,100,100]),"Deterministic");   % cost matrix for state (x, d)
param_base = system.addParam(param_base,"R",diag([1, 1, 1, 1])./(param_base.m.average^2),"Deterministic");   % cost matrix for input (u_theta, u_r, U_l, U_X)  
param_base = system.addParam(param_base,"P",diag([10000,10000,10000,10000]),"Deterministic");

% constant inputs
%u = zeros(3,param.Nt);    % u,U_l,U_X
%u = repmat([0;0;-300;0],param.Nt);
%u = 0;      % input for robot (N)
%U_l = 0;      % input for crane (wire control) (m/s^2)
%U_X = 1000;      % input for vessel position      (m/s^2)
%u0 = zeros(4,param_base.predict_steps.average);
%u0 = repmat([0;-param.bar_m*param.g;0;0],[1,param.Nt]);
%u0 = repmat([0;0;-param_base.bar_m.average*param_base.g.average;0],[1,param_base.predict_steps.average]);
%u0 = repmat([gravity_force*sin(pi/6);0;-gravity_force*cos(pi/6);0],[1,param_base.predict_steps.average]);
gravity_force = param_base.bar_m.average*param_base.g.average;
param_base = system.addParam(param_base,"u0",[gravity_force*sin(pi/6);0;-gravity_force*cos(pi/6);0],"Deterministic");  % TODO: repmat
param_base = system.addParam(param_base,"f0",[0; 0; -gravity_force; 0],"Deterministic");    % initial value of force input theta,r,l,X

%u0 = repmat([0;0;0;0],[1,param.Nu]);
%u0 = u_b;

%% simulation and planning
tic
seed_plan_list = 1:1;       % this seed list is planning (not implemented for MPPI)
seed_list = 1:1;            % this seed list is for simulation
param_base = system.addParam(param_base,"force_deterministic",false,"Deterministic");
param_base = system.addParam(param_base,"consider_collision",true,"Deterministic");
param_base = system.addParam(param_base,"visual_capture",true,"Deterministic");
q = zeros(length(param_base.q0.average),Nt,length(seed_list));
f = zeros(length(ub),Nt,length(seed_list));
u = f;
%i = 0;
%for seed = seed_list
%    i = i+1;
%    disp(string(i)+"/"+string(length(seed_list)))
    %[q(:,:,i),f(:,:,i),u(:,:,i),param_valid,F] = planningAndSimulateMPPI(u0,xd,param_base,seed_sample_list,seed_list,lb,ub);
    [q,f,u,param_valid,find_feasible_solution,~] = planningAndSimulateMPPI(param_base,seed_plan_list,seed_list);
    %x(:,:,i) = system.changeCoordinate(q(:,:,i),param_valid,xd);
%end
for i = 1:length(seed_simulate)
    x(:,:,i) = system.changeCoordinate(q(:,:,i),param_nominal);
end
max_energy_consumption = energyEvaluation(u(:,:,:),f(:,:,:),param_valid);
u_val = u;
toc

%% save
folder_name = "data/mppi_"+string(datetime('now','Format','yyyyMMdd/HH_mm_ss/'));
mkdir(folder_name);
save(folder_name+"simulation.mat")

%% Visualize
param = param_valid;
t_vec = dt:dt:dt*Nt;
snum_list = 1:length(seed_list);
%snum_list = [1];
visual.visualInit();
%visual.plotInputs(u,f,param,t_vec,[2,3],folder_name);
visual.plotRobotStates(q,param,t_vec,[7,8],folder_name,snum_list);
visual.plotRobotOutputs(x,xd,param,t_vec,[1 3; 2 4],folder_name,snum_list);
%visual.plotInputs(u,f,param,t_vec,[1,2;3,4],folder_name);
visual.plotInputsFB(u(:,:,1),u(:,:,:),f,param,t_vec,[1,2;3,4],folder_name,snum_list);
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
%visual.makeSnapsFB(q,q_nonFB,q_nominal,x,x_nonFB,x_nominal,param,t_vec,folder_name,[1],snum_list);
title("max energy consumption = "+string(max_energy_consumption))
%visual.makePathMovie(q,x,param,t_vec,folder_name,1,snum_list);

%% 
if param.visual_capture
    v = VideoWriter(folder_name+"sampling",'MPEG-4');
    %v = VideoWriter(filename,'MPEG-4');
    v.FrameRate = round(1/param.dt);
    open(v);
    writeVideo(v,F(1:end-1));
    close(v);
    disp("Animation : finish saving")
end
clear F