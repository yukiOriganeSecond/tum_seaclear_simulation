function q_target = astarTargetPathPlan(param)
    q_target = zeros(length(param.q0),param.Nt);
    % make map
    dx = 0.1;
    x_list = -4:dx:4;
    y_list = -1:dx:7;
    map = binaryOccupancyMap(length(x_list),length(y_list),1/dx,"grid");
    map.GridLocationInWorld = [x_list(1) y_list(1)];
    % set obstacle
    setOccupancy(map, param.obs_pos.', ones(size(param.obs_pos,2),1),"world")
    inflate(map, param.obs_size)
    % calculate start point
    start = system.changeCoordinate(param.q0,param);
    planner = plannerAStarGrid(map);
    % planning in orthogonal coordinate space
    path = plan(planner,world2grid(map,start([1 3],:).'),world2grid(map,param.xd([1 3],:).'));
    path = grid2world(map,path);
    % smoothing
    path(:,1) = smooth(path(:,1));
    path(:,2) = smooth(path(:,2));
    % fit time scale
    path_time = 1:param.Nt/length(path):param.Nt;
    Fx = griddedInterpolant(path_time, path);
    path_x = Fx(1:param.Nt);
    % add X 
    if size(param.xd,1) == 4
        param.xd = [param.xd; param.xd(1); 0];  % if Xd is not set, it is same as xd
    end
    FX = griddedInterpolant([1 param.Nt], [param.q0(5,1), param.xd(5,1)]);
    path_X = FX(1:param.Nt);
    % change coordinate
    path_theta = atan2(path_X.'-path_x(:,1), path_x(:,2));
    path_r = vecnorm([path_X.'-path_x(:,1), path_x(:,2)],2,2);
    q_target(1,:) = smooth(path_theta.',19,'sgolay');
    q_target(3,:) = path_r.';
    q_target(5,:) = path_X.';
    q_target(7,:) = path_r.';
end

