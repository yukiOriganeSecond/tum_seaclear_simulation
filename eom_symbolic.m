
syms x(t) d(t) theta(t) r(t) X(t) m m_bar M g

x(t) = X(t)-r(t)*sin(theta(t));
d(t) = r(t)*cos(theta(t));

K = 1/2*m*(diff(x,t)^2+diff(d,t)^2);
U = -m_bar*g*d;

L = simplify(K-U);

D1 = diff(L,diff(theta(t),t));
DD1 = diff(D1,t);
D2 = diff(L,theta);
eqn1 = simplify(DD1-D2);

D1 = diff(L,diff(r(t),t));
DD1 = diff(D1,t);
D2 = diff(L,r);
eqn2 = simplify(DD1-D2);

D1 = diff(L,diff(X(t),t));
DD1 = diff(D1,t);
D2 = diff(L,X);
eqn3 = simplify(DD1-D2);
