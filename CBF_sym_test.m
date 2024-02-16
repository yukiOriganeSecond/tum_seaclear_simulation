clear
syms r(t) theta(t) l(t) X(t) dr(t) dtheta(t) dl(t) dX(t) ddr(t) ddtheta(t) ddl(t) ddX(t) xo do a
syms m I_l m_bar g M
to_dt_old = [diff(r, t), diff(theta, t), diff(l,t), diff(X, t)];
to_dt_new = [dr, dtheta, dl, dX];
to_ddt_old = [diff(dr, t), diff(dtheta, t), diff(dl, t), diff(dX, t)];
to_ddt_new = [ddr, ddtheta, ddl, ddX];

to_all_old = [to_dt_old to_ddt_old];
to_all_new = [to_dt_new to_ddt_new];

%% CBF
y = [X-r*sin(theta)-xo; r*cos(theta)-do];
dy = diff(y,t);
dy = subs(dy, to_dt_old, to_dt_new);
%dy = [dX-dr*sin(theta)-r*dtheta*cos(theta); dr*cos(theta)-r*dtheta*sin(theta)];
ddy = diff(dy,t);
ddy = subs(ddy, to_all_old, to_all_new);
%ddy = subs(ddy, [diff(r, t), diff(theta, t), diff(X, t)], [dr, dtheta, dX]);
%ddy = [ddX-ddr*sin(theta)-r*ddtheta*cos(theta)-2*dr*dtheta*cos(theta)+r*dtheta^2*sin(theta);
%    ddr*cos(theta)-r*ddtheta*sin(theta)-2*dr*dtheta*sin(theta)-r*dtheta^2*cos(theta)];
syms gamma_0 gamma_1 gamma_2
h = y.'*y+2*(1/gamma_0+1/gamma_1)*dy.'*y+2/gamma_0/gamma_1*(ddy.'*y+dy.'*dy)-a^2;
q_bar = [r(t) theta(t) X(t) l(t) dr(t) dtheta(t) dl(t) dX(t) ddr(t) ddtheta(t) ddl(t) ddX(t)].';

nabla_h = gradient(h,q_bar);

%% system
syms mu [1 4]

% drag force
sigmoid_a = 100;                % constant value for sigmoid function
v_r = dr-dX*sin(theta);
v_theta = r*dtheta-dX*cos(theta);
dv_r = subs(diff(v_r, t), to_all_old, to_all_new);
dv_theta = subs(diff(v_theta, t), to_all_old, to_all_new);
%dv_r = ddr-ddX*sin(theta)-dX*dtheta*cos(theta);
%dv_theta = dr*dtheta+r*ddtheta-ddX*cos(theta)+dX*dtheta*sin(theta);

eta_r = mu(1)*(2/(1+exp(-sigmoid_a*v_r))-1)*v_r;
eta_theta = mu(2)*(2/(1+exp(-sigmoid_a*v_theta))-1)*v_theta;
eta_l = mu(3)*dl;
eta_X = mu(4)*dX;

deta_r = subs(diff(eta_r, t), to_all_old, to_all_new);
deta_theta = subs(diff(eta_theta, t), to_all_old, to_all_new);
deta_l = subs(diff(eta_l, t), to_all_old, to_all_new);
deta_X = subs(diff(eta_X, t), to_all_old, to_all_new);

%deta_r = mu(1)*2*a*dv_r*v_r*exp(-a*v_r)/(1+exp(-a*v_r))^2+(2/(1+exp(-a*v_r))-1)*dv_r;
%deta_theta = mu(2)*2*a*dv_theta*v_theta*exp(-a*v_theta)/(1+exp(-a*v_theta))^2+(2/(1+exp(-a*v_theta))-1)*dv_theta;
%deta_l = mu(3)*ddl;
%deta_X = mu(4)*ddX;

% input
syms f_r(t) f_theta(t) F_l(t) F_X(t)
syms T_r T_theta T_l T_X

% Inertia Matrix for third derivative
sys_M_mat = [
    m+I_l, 0, 0, -m*sin(theta);
    0, m*r^2, 0, -m*r*cos(theta);
    1, 0, -1, 0;
    0, 0, 0, M;
    ];
% self vector
sys_f_ddt = [
    m*ddX*sin(theta)+m*r*dtheta^2+m_bar*g*r*cos(theta)-eta_r-eta_l;
    m*ddX*r*cos(theta)-2*m*r*dr*dtheta-m_bar*g*r*sin(theta)-r*eta_theta;
    0;
    -eta_X
    ];
sys_f = subs(diff(sys_f_ddt, t), to_all_old, to_all_new);
sys_f = subs(sys_f, diff(ddX, t), 0);   % this term is move to left hand side
sys_f = sys_f + [
    -1/T_r*f_r-1/T_l*F_l;
    -2*m*r*dr*ddtheta-1/T_theta*f_theta;
    0;
    -1/T_X*F_X
    ];
%f = [
%    -m*ddX*dtheta*cos(theta)+m*dr*dtheta^2+2*m*r*dtheta*ddtheta-m_bar*g*dtheta*sin(theta)-deta_r-deta_l-1/T_r*f_r-1/T_l*F_l;
%    -2*m*r*dr*ddtheta+m*ddX*r*cos(theta)-m*ddX*r*dtheta*sin(theta)-2*m*dr^2*dtheta-2*m*r*ddr*dtheta-2*m*r*dr*ddtheta-m_bar*(g*dr*sin(theta)+g*r*dtheta*cos(theta))+dr*(-eta_theta+f_theta)+r*(-deta_theta+-1/T_theta*f_theta);
%    0;
%    -deta_X-1/T_X*F_X
%    ];
sys_g = [
    1/T_r, 0, 1/T_l, 0;
    0, r*1/T_theta, 0, 0;
    0, 0, 0, 0;
    0, 0, 0, 1/T_X
    ];
bar_f = sys_M_mat\sys_f;
bar_g = sys_M_mat\sys_g;

%% calc constraint

bar_f_vec = [dr dtheta dl dX ddr ddtheta ddl ddX bar_f.'].';
bar_g_mat = [zeros(8,4); bar_g];
Lfh = nabla_h.'*bar_f_vec;
Lgh = nabla_h.'*bar_g_mat;

gamma2_h = gamma_2 * h;

%% parameter substitution
param_name = [m, I_l, m_bar, g, M, gamma_0, gamma_1, gamma_2, T_r, T_theta, T_l, T_X, xo, do, a, mu];
param_val = [120 30 90 9.8 1075 1 1 1 0.2 0.2 0.5 1.0 0 5 1 120 120 300 400];
Lfh = subs(Lfh, param_name,param_val);
Lgh = subs(Lgh, param_name,param_val);
gamma2_h = subs(gamma2_h, param_name,param_val);

%% state substitution
state_name = [r(t), theta(t) l(t) X(t), dr(t), dtheta(t), dl(t), dX(t), ddr(t), ddtheta(t), ddl(t), ddX(t), f_r(t), f_theta(t), F_l(t), F_X(t)];
state_val = [3.6 0 0 0 0.01 0 0 0 0 0 0 0 0 0 0 0];% 0 0 0 0 0 0 0 0 0 0 0 0];
Lfh = double(subs(Lfh, state_name, state_val));
Lgh = double(subs(Lgh, state_name, state_val));
gamma2_h = double(subs(gamma2_h, state_name,state_val));
