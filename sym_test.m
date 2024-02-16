
syms x(t)
f = x^2;

b = func(f);
disp(b)

function a = func(f)
    syms x
    a =  subs(f,x,3);
end