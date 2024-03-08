function tb_result = calcTtest2(val,x_list)
    Nsim = length(x_list);
    h_result = zeros(3,Nsim);
    p_result = zeros(3,Nsim);
    for sim_index = 1:Nsim
        m = 0;
        for method_index = [1,3,4]
            m = m+1;
            [h,p] = ttest2(val(:,2,sim_index),val(:,method_index,sim_index),'Vartype','unequal',"Tail","left");
            h_result(m,sim_index) = h;
            p_result(m,sim_index) = p;
        end
    end

    tb_result = table(x_list.',h_result(1,:).',p_result(1,:).',h_result(2,:).',p_result(2,:).',h_result(3,:).',p_result(3,:).');
    tb_result.Properties.VariableNames = ["sim","h RA-SAA", "p RA-SAA","h Local", "p Local","h MPPI", "p MPPI"];

end

