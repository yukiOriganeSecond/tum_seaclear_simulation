
Y1 = [0 0.4 0.7 1.0 1.0];
Y2 = [0 0 0 0 0.1];

X = categorical(["1%","5%","10%","20%","50%"]);
X = reordercats(X,["1%","5%","10%","20%","50%"]);
bar(X,[Y1;Y2])

legend("RA-SAA","RA-SAA-PID")
xlabel("parameter uncertainty")
ylabel("infeasible rate")