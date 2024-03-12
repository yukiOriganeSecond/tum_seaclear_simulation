%%
dx = 0.1;

x_list = -4:dx:4;
y_list = -1:dx:7;

obs_pos = [0;3];
obs_size = 1.0;

resolution = 1/dx;
map = binaryOccupancyMap(length(x_list),length(y_list),resolution,"grid");
map.GridLocationInWorld = [-4 -1];

setOccupancy(map, obs_pos.', ones(size(obs_pos,2),1))
inflate(map, obs_size)

start = [-2 6];
goal = [2 1];
planner = plannerAStarGrid(map);
path = plan(planner,world2grid(map,start),world2grid(map,goal));
path = grid2world(map,path);
show(planner)
axis xy

%% smoothing
Nt = 200;
path_time = 1:Nt/length(path):Nt;
Fx = griddedInterpolant(path_time, path);
path_x = Fx(1:Nt);
visual.visualInit();
figure
plot(1:Nt, path_x(:,:))
xlabel("timestep")
ylabel("target point")

%%
figure
show(map)
hold on
j = 1
obspos_(j,:) = [(obs_pos(:,j)-obs_size(1,j)).', (2*obs_size(:,j))*[1 1]];
rectangle('Position',obspos_(j,:),'Curvature',[1 1],'EdgeColor','r');
plot(path(:,1),path(:,2))
grid on
axis ij