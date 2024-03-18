classdef CBF
    %CBF このクラスの概要をここに記述
    %   詳細説明をここに記述
    
    properties
        Lfh             % all symbolic variable
        Lgh
        gamma2_h
        Lfh_with_param  % substituted parameter and not state variables
        Lgh_with_param
        gamma2_h_with_param
    end
    
    methods
        function obj = CBF()
            syms r(t) theta(t) l(t) X(t) dr(t) dtheta(t) dl(t) dX(t) ddr(t) ddtheta(t) ddl(t) ddX(t) xo do a
            syms f_r(t) f_theta(t) F_l(t) F_X(t)
            syms gamma_0 gamma_1 gamma_2
            syms m I_l m_bar g M
            syms T_r T_theta T_l T_X
            syms mu_ [1 4]
            to_dt_old = [diff(r, t), diff(theta, t), diff(l,t), diff(X, t)];
            to_dt_new = [dr, dtheta, dl, dX];
            to_ddt_old = [diff(dr, t), diff(dtheta, t), diff(dl, t), diff(dX, t)];
            to_ddt_new = [ddr, ddtheta, ddl, ddX];
            
            to_all_old = [to_dt_old to_ddt_old];
            to_all_new = [to_dt_new to_ddt_new];
            
            % CBF
            y = [X-r*sin(theta)-xo; r*cos(theta)-do];
            dy = diff(y,t);
            dy = subs(dy, to_dt_old, to_dt_new);
            ddy = diff(dy,t);
            ddy = subs(ddy, to_all_old, to_all_new);
            h = y.'*y+2*(1/gamma_0+1/gamma_1)*dy.'*y+2/gamma_0/gamma_1*(ddy.'*y+dy.'*dy)-a^2;
            %h = y.'*y+2*(1/gamma_0+1/gamma_1)*dy.'*y+2/gamma_0/gamma_1*(ddy.'*y)-a^2;
            q_bar = [r(t) theta(t) l(t) X(t) dr(t) dtheta(t) dl(t) dX(t) ddr(t) ddtheta(t) ddl(t) ddX(t)].';
            
            nabla_h_q = gradient(h,q_bar);
            
            % drag force
            sigmoid_a = 100;                % constant value for sigmoid function
            v_r = dr-dX*sin(theta);
            v_theta = r*dtheta-dX*cos(theta);
            
            eta_r = mu_(1)*(2/(1+exp(-sigmoid_a*v_r))-1)*v_r^2;
            eta_theta = mu_(2)*(2/(1+exp(-sigmoid_a*v_theta))-1)*v_theta^2;
            eta_l = mu_(3)*dl;
            eta_X = mu_(4)*dX;
            
            % Inertia Matrix for third derivative
            sys_M_mat = [
                m+I_l, 0, 0, -m*sin(theta);
                0, m*r^2, 0, -m*r*cos(theta);
                1, 0, -1, 0;
                0, 0, 0, M;
                ];
            % self vector
            sys_f_ddt = [
                m*r*dtheta^2+m_bar*g*cos(theta)-eta_r-eta_l+f_r+F_l;
                2*m*r*dr*dtheta-m_bar*g*r*sin(theta)-r*eta_theta+r*f_theta;
                0;
                -eta_X+F_X
                ];
            Tinv = diag([1/T_r, 1/T_theta, 1/T_l, 1/T_X]);
            F = [f_r(t); f_theta(t); F_l(t); F_X(t)];
            d_ddq_dF = jacobian(sys_M_mat\sys_f_ddt, F);
            nabla_h_x = nabla_h_q.'*blkdiag(eye(8),d_ddq_dF);  % transfer from q to x
            %nabla_h_x(9:12) = nabla_h_q(9:12).'*d_ddq_dF;    % change q to x by chain rule, d_q/dF = d_dq/dF = 0 
            sys_f = [dr; dtheta; dl; dX; sys_M_mat\sys_f_ddt; -Tinv*F];
            %sys_g = [
            %    1/T_r, 0, 1/T_l, 0;
            %    0, r*1/T_theta, 0, 0;
            %    0, 0, 0, 0;
            %    0, 0, 0, 1/T_X
            %    ];
            sys_g = [
                0, 1/T_r, 0, 0;
                1/T_theta, 0, 0, 0;
                0, 0, 1/T_l, 0;
                0, 0, 0, 1/T_X
                ];
            bar_f = sys_f;
            bar_g = [zeros(8,4); sys_g];
            
            %% calc constraint
            
            %bar_f_vec = [dr dtheta dl dX ddr ddtheta ddl ddX bar_f.'].';
            %bar_g_mat = [zeros(8,4); bar_g];
            obj.Lfh = nabla_h_x*bar_f;
            obj.Lgh = nabla_h_x*bar_g;
            
            obj.gamma2_h = gamma_2 * h;
        end
        
        function obj = substituteParameters(obj,param)
            syms gamma_0 gamma_1 gamma_2
            syms m I_l m_bar g M xo do a
            syms T_r T_theta T_l T_X
            syms mu_ [1 4]
            param_names_1 = [gamma_0, gamma_1, gamma_2];
            param_vals_1 = param.gamma;
            param_names_2 = [m, I_l, m_bar, M, g];
            param_vals_2 = [param.m, param.I_l, param.bar_m, param.M, param.g];
            param_names_3 = [T_theta, T_r, T_l, T_X, mu_];
            param_val_3 = [param.T.', param.mu_r(1), param.mu_theta(1), param.Mu_l(2), param.Mu_X(2)];
            param_names_4 = [xo, do, a];
            obj.Lfh_with_param = sym('Lfh', [size(param.obs_pos,2), 1]);
            obj.Lgh_with_param = sym('Lgh', [size(param.obs_pos,2), 4]);
            obj.gamma2_h_with_param = sym('gh', [size(param.obs_pos,2), 1]);
            for j = 1:size(param.obs_pos,2)
                param_val_4 = [param.obs_pos(1,j), param.obs_pos(2,j), param.obs_size(1,j)];
                obj.Lfh_with_param(j,1) = subs(obj.Lfh, [param_names_1, param_names_2, param_names_3, param_names_4], [param_vals_1, param_vals_2, param_val_3, param_val_4]);
                obj.Lgh_with_param(j,:) = subs(obj.Lgh, [param_names_1, param_names_2, param_names_3, param_names_4], [param_vals_1, param_vals_2, param_val_3, param_val_4]);
                obj.gamma2_h_with_param(j,1) = subs(obj.gamma2_h, [param_names_1, param_names_2, param_names_3, param_names_4], [param_vals_1, param_vals_2, param_val_3, param_val_4]);
            end
        end

        function obj = substituteParametersWithoutObstacle(obj,param)
            syms gamma_0 gamma_1 gamma_2
            syms m I_l m_bar g M
            syms T_r T_theta T_l T_X
            syms mu_ [1 4]
            param_names_1 = [gamma_0, gamma_1, gamma_2];
            param_vals_1 = param.gamma;
            param_names_2 = [m, I_l, m_bar, M, g];
            param_vals_2 = [param.m, param.I_l, param.bar_m, param.M, param.g];
            param_names_3 = [T_theta, T_r, T_l, T_X, mu_];
            param_val_3 = [param.T.', param.mu_r(1), param.mu_theta(1), param.Mu_l(2), param.Mu_X(2)];
            %param_names_4 = [xo, do, a];
            obj.Lfh_with_param = sym('Lfh', [size(param.obs_pos,2), 1]);
            obj.Lgh_with_param = sym('Lgh', [size(param.obs_pos,2), 4]);
            obj.gamma2_h_with_param = sym('gh', [size(param.obs_pos,2), 1]);
            for j = 1:size(param.obs_pos,2)
                %param_val_4 = [param.obs_pos(1,j), param.obs_pos(2,j), param.obs_size(1,j)];
                obj.Lfh_with_param(j,1) = subs(obj.Lfh, [param_names_1, param_names_2, param_names_3], [param_vals_1, param_vals_2, param_val_3]);
                obj.Lgh_with_param(j,:) = subs(obj.Lgh, [param_names_1, param_names_2, param_names_3], [param_vals_1, param_vals_2, param_val_3]);
                obj.gamma2_h_with_param(j,1) = subs(obj.gamma2_h, [param_names_1, param_names_2, param_names_3], [param_vals_1, param_vals_2, param_val_3]);
            end
        end
        
        function obj = substituteObstacle(obj,param,pos_est,a_est)
            syms xo do a
            param_names = [xo, do, a];
            for j = 1:size(param.obs_pos,2)
                param_vals = [pos_est(1,j), pos_est(2,j), a_est(1,j)];
                obj.Lfh_with_param(j,1) = subs(obj.Lfh_with_param(j,1), param_names, param_vals);
                obj.Lgh_with_param(j,:) = subs(obj.Lgh_with_param(j,:), param_names, param_vals);
                obj.gamma2_h_with_param(j,1) = subs(obj.gamma2_h_with_param(j,1), param_names, param_vals);
            end
        end

        function [A,b,h] = calculateConstraint(obj,q,q_ddot,f)
            syms r(t) theta(t) l(t) X(t) dr(t) dtheta(t) dl(t) dX(t) ddr(t) ddtheta(t) ddl(t) ddX(t)
            syms f_r(t) f_theta(t) F_l(t) F_X(t)
            state_name_1 = [theta(t), dtheta(t), l(t), dl(t), X(t), dX(t), r(t), dr(t), ddtheta(t), ddl(t), ddX(t), ddr(t)];
            state_val_1 = [q.', q_ddot.'];
            state_name_2 = [f_theta(t), f_r(t), F_l(t), F_X(t)];
            state_val_2 = f.';
            Lfh_ = subs(obj.Lfh_with_param, [state_name_1, state_name_2], [state_val_1,state_val_2]);
            Lgh_ = subs(obj.Lgh_with_param, [state_name_1, state_name_2], [state_val_1,state_val_2]);
            gamma2_h_ = subs(obj.gamma2_h_with_param, [state_name_1, state_name_2], [state_val_1,state_val_2]);
            A = -double(Lgh_);
            b = double(Lfh_)+double(gamma2_h_);
            h = double(gamma2_h_);
        end
    end
end

