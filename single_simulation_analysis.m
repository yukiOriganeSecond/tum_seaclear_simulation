
q_nominal = zeros(length(param_nominal.q0),param_nominal.Nt);

for t = 1:param_nominal.Nt
    q_nominal(:,t) = (param_nominal.qd-param_nominal.q0)/param_nominal.Nt*t+param_nominal.q0;
    u_nominal(:,t) = param_nominal.bar_m*param_nominal.g*[sin(q(1,t,1));0;-cos(q(1,t,1));0];
    et = q_nominal([1,7,3,5],t) - q([1,7,3,5],t,1);  % error at time t
    dt_et = q_nominal([2,8,4,6],t) - q([2,8,4,6],t,1);
    %+ param.sensing_noise([1,7,3,5],1).*dW;
    u_nominal(:,t) = u_nominal(:,t) + param_nominal.kp.*et + param_nominal.kd.*dt_et;% + param.ki.*integral_et;
end

x_nominal(:,:) = system.changeCoordinate(q_nominal(:,:),param_nominal);
folder_name = "fig/";

visual.makeSnapsFB(q,q_nominal,q_nominal,x,x_nominal,x_nominal,param_nominal,t_vec,folder_name,[1],1:Nsim);
visual.plotRobotOutputsFB(x,param_nominal.xd,x_nominal,x_nominal,param_nominal,t_vec,[1,3;2,4],folder_name,1:Nsim);
visual.plotRobotStatesFB(q,q_nominal,q_nominal,param_nominal,t_vec,[1,5,7;2,6,8],folder_name,1:Nsim);
visual.plotInputs(u,f,param_nominal,t_vec,[1,2;3,4],folder_name,1:Nsim);
visual.plotInputsFB(u_nominal(:,:),u(:,:,:),u,param_nominal,t_vec,[1,2;3,4],folder_name,1:Nsim);

%visual.makePathMovie(q,x,param_nominal,t_vec,folder_name,1,1:Nsim);