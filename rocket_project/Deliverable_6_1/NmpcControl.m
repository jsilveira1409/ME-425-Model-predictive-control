classdef NmpcControl < handle
    
    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
    methods
        function obj = NmpcControl(rocket, tf)
           
            import casadi.*
            
            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
            
            % Default state and input constraints
            ubx = inf(nx, 1);
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            %Continuous time dynamics
            rocket_continuous = @(x,u) rocket.f(x,u);
            %Discretisation
            rocket_discrete = @(x,u) RK4(x,u,rocket.Ts,rocket_continuous);
            
            % input
            d1   = U_sym(1,:);
            d2   = U_sym(2,:);
            Pavg = U_sym(3,:);
            Pdiff = U_sym(4,:);

            % states
            wx      = X_sym(1,:);
            wy      = X_sym(2,:);
            wz      = X_sym(3,:);
            alpha   = X_sym(4,:);
            beta    = X_sym(5,:);
            gamma   = X_sym(6,:);
            vx      = X_sym(7,:);
            vy      = X_sym(8,:);
            vz      = X_sym(9,:);
            x       = X_sym(10, :);
            y       = X_sym(11,:);
            z       = X_sym(12,:);
            
            % reference
            x_ref    = ref_sym(1, 1);
            y_ref    = ref_sym(2, 1);
            z_ref    = ref_sym(3, 1);
            gamma_ref = ref_sym(4, 1);

            % info of system for trim and terminal set
            % TODO : look into this
            rocket = Rocket(rocket.Ts);
            [xs, us] = rocket.trim();
            linear_sys = rocket.linearize(xs, us);
            discrete_linear_sys = c2d(linear_sys, rocket.Ts);

            Q = diag([0.01 0.01 1 ...
                      1    1    10 ...
                      10   10   1 ...
                      40   40   50]);
            R = diag([0.01 0.01 0.005 0.01]);
            % LQR controller for unconstrained system
            [K, Qf, ~] = dlqr(discrete_linear_sys.A, discrete_linear_sys.B, Q, R);

            % objective function
            o = 0;
            eq_constr = [ ; ];
            ineq_constr = [;];
            % constraint bounds
            beta_max = deg2rad(80);
            d1_max = deg2rad(15);    %CHECK SHOULD BE RAD
            d2_max = deg2rad(15);
            Pavg_min = 50;
            Pavg_max = 80;
            Pdiff_max = 20;


            for i = 1:N-1
                o = o + (X_sym(:,i)- [ xs(1:5) ; gamma_ref ; xs(7:9) ; x_ref ; y_ref ; z_ref ])' * Q * (X_sym(:,i)- [ xs(1:5) ; gamma_ref ; xs(7:9) ; x_ref ; y_ref ; z_ref ]);
                o = o + ((U_sym(:,i)-us)' * R * (U_sym(:,i)-us));
                
                % constraints
                eq_constr = [eq_constr; X_sym(:,i+1) - rocket_discrete(X_sym(:,i), U_sym(:,i))];
                ineq_constr = [ineq_constr; -beta_max - X_sym(5,i)];
                ineq_constr = [ineq_constr; -beta_max + X_sym(5,i)];
                ineq_constr = [ineq_constr; -d1_max - U_sym(1,i)];
                ineq_constr = [ineq_constr; -d1_max + U_sym(1,i)];
                ineq_constr = [ineq_constr; -d2_max - U_sym(2,i)];
                ineq_constr = [ineq_constr; -d2_max + U_sym(2,i)];
                ineq_constr = [ineq_constr; Pavg_min - U_sym(3,i)];
                ineq_constr = [ineq_constr; -Pavg_max + U_sym(3,i)];
                ineq_constr = [ineq_constr; -Pdiff_max - U_sym(4,i)];
                ineq_constr = [ineq_constr; -Pdiff_max + U_sym(4,i)];
            end
            o = o +(X_sym(:,N)-[ xs(1:5) ; gamma_ref ; xs(7:9) ; x_ref ; y_ref ; z_ref ])'*Qf*(X_sym(:,N)-[ xs(1:5) ; gamma_ref ; xs(7:9) ; x_ref ; y_ref ; z_ref ]);
            eq_constr = [eq_constr; X_sym(:,1) - x0_sym];
            
            cost = o;

         % i don't get the following  :
            % For box constraints on state and input, overwrite entries of
            % lbx, ubx, lbu, ubu defined above
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----
            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0, ref)
            
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
            
            % Additional evaluation
            % Complete trajectory
            % % X_opt = full(reshape(nlp_x(idx_X(1):idx_X(2)), obj.nx, obj.N));
            % % U_opt = full(reshape(nlp_x(idx_U(1):idx_U(2)), obj.nu, obj.N - 1));
            % %
            % % cost_opt = full(sol.f);
            % % constr_opt = full(sol.g);
            % %
            % % stats = obj.solver.stats;
        end
        
        function solve(obj, x0, ref)
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end

