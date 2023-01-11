classdef MpcControl_z < MpcControlBase
    properties
        A_bar, B_bar, C_bar % Augmented system for disturbance rejection
        L                   % Estimator gain for disturbance rejection
    end
    
    methods
        function mpc = MpcControl_z(sys, Ts, H)
            mpc = mpc@MpcControlBase(sys, Ts, H);
            
            [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
        end
        
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   d_est        - disturbance estimate
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing
            
            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.3)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar(1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % hyperparameters
            Q = [30   0;
                 0  50];
            R = 0.1 * eye(nu);

            % no state constraint
            F = [1 0 ; 0 1 ;-1 0 ; 0 -1 ];
            f = [inf ; inf ; inf ; inf];
            
            % input constraint
            M = [1; -1];
            % the 56.6667 comes from the fact that we trimmed the system.
            m = [(80 - 56.6667); -(50 - 56.6667)];

            % unconstrained system LQR controller
            [K, Qf, ~] = dlqr(mpc.A, mpc.B, Q, R);
            K = -K;

            % maximal invariant set
            Xf = polytope([F; M*K],[f; m]);

            
            Acl = [mpc.A + mpc.B * K];
            while 1
               prevXf = Xf;
               [T,t] = double(Xf);
               preXf = polytope(T * Acl, t);
               Xf = intersect(Xf, preXf);
               if isequal(prevXf, Xf)
                   break
               end
            end
            [Ff,ff] = double(Xf);
            
            % yalmip optimization
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1) + mpc.B*d_est) + (M*U(:,1) <= m);
            obj = (U(:,1)-u_ref)'*R*(U(:,1)-u_ref);
            
            for i = 2:N-1
                con = con + (X(:,i+1) == mpc.A*X(:,i) + mpc.B*U(:,i) + mpc.B*d_est);
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
                obj = obj + (X(:,i)-x_ref)'*Q*(X(:,i)-x_ref) + (U(:,i)-u_ref)'*R*(U(:,i)-u_ref);
            end
            
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + (X(:,N) - x_ref)' * Qf * (X(:,N) - x_ref);
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref, d_est}, {U(:,1), X, U});
        end
        
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);
            
            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.3)
            ref = sdpvar;
            
            % Disturbance estimate (Ignore this before Part 5)
            d_est = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];
            Rs = 1;

            % no state constraint
            F = [1 0 ; 0 1 ;-1 0 ; 0 -1 ];
            f = [inf ; inf ; inf ; inf];
            
            % input constraint
            M = [1; -1];
            % TODO check this : the 56.6667 comes from the fact that we trimmed the system.
            m = [(80 ); (50)];

            % definition of the function we want to optimize
            obj = us' * Rs * us;
            con = [(eye(nx) - mpc.A) -mpc.B; mpc.C  0] * [xs ;us] ...
                    ==  [mpc.B*d_est; ref];
            
            % TODO: i don't get this line
            con = con + (F * xs <= f) + (M * us <= m);

            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
        end
        
        
        % Compute augmented system and estimator gain for input disturbance rejection
        function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
            
            %%% Design the matrices A_bar, B_bar, L, and C_bar
            %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
            %%% converges to the correct state and constant input disturbance
            %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            ny = size(mpc.C,1)
            nu = size(mpc.B, 2)
            nx = size(mpc.A,1)
            
            A_bar = [mpc.A,         mpc.B;
                     zeros(1,nx),   1];
            B_bar = [mpc.B;         zeros(1,nu)];
            C_bar = [mpc.C,         zeros(ny,1)];

            % TODO : check the [] values
            L = -place(A_bar', C_bar', [0.1, 0.5, 0.9])';
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
        
        
    end
end
