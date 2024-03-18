function [pos_est,size_est] = estimateObstacleInformation(param_nominal, param_real, qt)
% change estimation error as distance between robot and obstacle region
    xt = system.changeCoordinate(qt,param_nominal,zeros(4,1));
    x0 = system.changeCoordinate(param_nominal.q0,param_nominal,zeros(4,1));
    dist_t = vecnorm(param_real.obs_pos(:,:)-xt([1,3],1),2,1)-param_real.obs_size(1,:);
    dist_0 = vecnorm(param_real.obs_pos(:,:)-x0([1,3],1),2,1)-param_real.obs_size(1,:);
    rho = dist_t/dist_0;
    rho = rho.*(rho>0); % if rho<0, rho equal to be 0
    pos_est = param_nominal.obs_pos.*rho + param_real.obs_pos.*(1-rho);
    size_est = param_nominal.obs_size.*rho + param_real.obs_size.*(1-rho);
end

