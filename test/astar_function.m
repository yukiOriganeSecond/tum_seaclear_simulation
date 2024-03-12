
clear
load("test/astar_test_param.mat")
q = system.astarTargetPathPlan(param_nominal);
x = system.changeCoordinate(q,param_nominal,param_nominal.xd);

visual.visualInit();
visual.makeSnapsWithPoints(q,x,param_nominal,scenario_use,t_vec,"test/",[1],[1])

visual.plotRobotStates(q,param_nominal,t_vec,[1 7 5],"test/");