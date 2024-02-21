function x = changeCoordinate(q,param,xd_leng_ref)
    arguments
        q
        param
        xd_leng_ref = zeros(4,1);   % if not receive xd, assume x = [x; dx; d; dd];
    end
    
    x = zeros(length(xd_leng_ref),size(q,2));
    sq = sin(q(1,:)); cq = cos(q(1,:));
    x([2,4],:) = q(8,:).*[-sq;cq]+q(7,:).*q(2,:).*[-cq;-sq]+[q(6,:);zeros(1,size(q,2))];
    x([1,3],:) = q(7,:).*[-sq;cq]+[q(5,:);zeros(1,size(q,2))];
    if length(xd_leng_ref)==6
        x([5,6],:) = q([5,6],:);    % return X,dX also
    end
    %for t = 1:param.Nt
        %theta = q(1,t);
        %theta_dot= q(2,t);
        %l = q(3,t);
        %l_dot = q(4,t);
        %X = q(5,t);
        %X_dot = q(6,t);
        %r = q(7,t);
        %r_dot = q(8,t);
        %x(2,t) = -r_dot*sin(theta)-r*theta_dot*cos(theta)+X_dot;
        %x(4,t) = r_dot*cos(theta)-r*theta_dot*sin(theta);
        %x(1,t) = -r*sin(theta)+X;
        %x(3,t) = r*cos(theta);
    %end
end