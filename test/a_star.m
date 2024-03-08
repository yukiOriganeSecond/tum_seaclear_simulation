
dx = 2;

y_list = -1:dx:7;
x_list = -4:dx:4;

X = repmat(x_list,length(y_list),1);
Y = repmat(y_list.',1,length(x_list));

pos = [X(:).'; Y(:).'];
Nnode = length(pos);
status = repmat("ready",1,Nnode);

DX = pos(1,:)-pos(1,:).';
DY = pos(2,:)-pos(2,:).';
Dist = sqrt(DX.^2+DY.^2);
Adj = (Dist<(dx*1.5)) - eye(Nnode);

obs_pos = [0;3];
obs_size = 1.0;
violate_index = prod(abs(pos(:,:)-obs_pos)<obs_size,1);
status(violate_index==1) = "violate";

status(10) = "open";    % start node
status(16) = "goal";
color_base = ["#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE","#A2142F","#0000FF","#00FF00","#FF0000","#FF00FF","#00FFFF"];

real_cost = zeros(Nnode,1);
estimate_cost = zeros(Nnode,1);
parent_node = zeros(Nnode,1);

while 1
    
end

for i = 1:Nnode
    if (status(i)=="ready")
        color = color_base(1);
    elseif (status(i)=="violate")
        color = color_base(2);
    elseif (status(i)=="start")
        color = color_base(3);
    elseif (status(i)=="goal")
        color = color_base(4);
    else
        color = color_base(5);
    end
    plot(pos(1,i),pos(2,i),'o','Color',color)
    hold on
end
axis ij
