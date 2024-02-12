
t = -0.3:0.1:0.3;
d_list = [-0.1,0,0.1,0.2,0.3];
alpha = 0.05;

for d = d_list
    f = t+1/alpha*max(-d-t,0);
    hold on
    plot(t,f)
end
legend("$d = "+string(d_list)+"$",'Interpreter','latex')
ylim([-0.5 2])
xlabel("$s$",'Interpreter','latex')
ylabel("$s+\frac{1}{\alpha}{\rm max}(-d-s,0)$",'Interpreter','latex')

%% 
avg_d = 4;
sigma_d = 2.5;
M = 10;
rng(4)
dm = sigma_d*randn(M,1)+avg_d;
t = -1:0.001:1;
alpha_list = [0.01,0.02,0.05,0.10,0.20,0.50];
for alpha = alpha_list
    f = t+1/alpha*mean(max(-dm-t,0),1);
    hold on
    plot(t,f)
end
xlabel("$s$",'Interpreter','latex')
ylabel("$s+\frac{1}{\alpha}{\rm max}(-d-s,0)$",'Interpreter','latex')
legend("$\alpha = "+string(alpha_list)+"$",'Interpreter','latex')
ylim([-1 1])

figure
histogram(dm,[-1:1:10])