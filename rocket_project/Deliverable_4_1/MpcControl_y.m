classdef MpcControl_y < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % hyperparameters
            
            %Q = 1 * diag([3 1 3 1]);
            %R = 1 * eye(nu);

            Q = [10  0  0  0;
                0  1  0  0;
                0  0  3  0;
                0  0  0  20];
            R = 0.1 * eye(nu);

            % state constraints

            F = [1  0  0  0;
                 0  1  0  0;
                 0  0  1  0;
                 0  0  0  1;
                -1  0  0  0;
                 0 -1  0  0;
                 0  0 -1  0;
                 0  0  0 -1];

            f = [inf; deg2rad(7); inf; inf; 
                 inf; deg2rad(7); inf; inf];

            % input constraints
            M = [1; -1];
            m = [deg2rad(15); deg2rad(15)];
            
            % unconstrained system LQR controller
            [K, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);
            K = -K;

            % maximal invariant set
            Xf = polytope([F; M*K],[f; m]);
            
            
            Acl = [mpc.A + mpc.B * K];
            
            while 1
               prevXf = Xf;
               [T,t] = double(Xf);
               preXf = polytope(T*Acl, t);
               Xf = intersect(Xf, preXf);
               if isequal(prevXf, Xf)
                   break
               end
            end
            [Ff,ff] = double(Xf);
            
            
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (M*U(:,1) <= m);
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            
            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + (X(:,N)-x_ref)'*Qf*(X(:,N)-x_ref);
            
            
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];

            Rs = 1;

                        % Contraints on the steady-state input target
            m = [deg2rad(15); deg2rad(15)];
            M = [1; -1];

            % Constraints on the steady-state target    
            % TODO: same as before right?
            f = [inf; deg2rad(7); inf; inf; inf; deg2rad(7); inf; inf];
            F = [1  0  0  0;
                 0  1  0  0;
                 0  0  1  0;
                 0  0  0  1;
                -1  0  0  0;
                 0 -1  0  0;
                 0  0 -1  0;
                 0  0  0 -1];

            % definition of the function we want to optimize
            obj = us' * Rs * us;
            con = [(eye(nx) - mpc.A) -mpc.B; mpc.C  0] * [xs ;us] ...
                    ==  [zeros(nx,1); ref];
            
            % TODO: i don't get this line
            con = con + (F * xs <= f) + (M * us <= m);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
