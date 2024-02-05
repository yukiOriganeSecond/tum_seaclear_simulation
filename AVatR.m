
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