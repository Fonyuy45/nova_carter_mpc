%% nmpc_controller.m
% Nonlinear Model Predictive Controller for differential drive robot
%
% PURPOSE:
%   Optimal trajectory tracking controller using receding horizon optimization
%   Solves a finite-horizon optimal control problem at each time step
%
% MATHEMATICAL FORMULATION:
%   At each time t, solve:
%   
%   min  Σ [||x_k - x_ref||²_Q + ||u_k||²_R + ||Δu_k||²_S] + ||x_N - x_ref||²_Qf
%   u    k=0 to N-1
%   
%   subject to:
%     x_{k+1} = f(x_k, u_k)           (system dynamics)
%     u_min ≤ u_k ≤ u_max              (input constraints)
%     |u_k - u_{k-1}| ≤ Δu_max        (rate constraints)
%   
%   where:
%     x = [x, y, θ]^T                 (state: position + heading)
%     u = [v, ω]^T                    (control: linear + angular velocity)
%     Q, R, S, Qf                     (weight matrices)
%     N                               (prediction horizon)
%
% ALGORITHM:
%   Sequential Quadratic Programming (SQP) via MATLAB's fmincon
%   - Iteratively solves quadratic approximations
%   - Handles nonlinear dynamics and constraints
%   - Warm-started with previous solution
%
% INTEGRATION WITH EKF:
%   - Takes current state estimate from EKF as initial condition
%   - Outputs optimal control command [v, ω]
%   - Controller runs at same rate as EKF (typically 10-50 Hz)

classdef nmpc_controller
    properties
        params          % Robot parameters (nova_carter_params)
        model           % Kinematic model (differential_drive_model)
        
        % MPC parameters
        N               % Prediction horizon (number of steps)
        dt              % Time step (seconds)
        
        % Cost function weights
        Q               % State tracking weight (3x3 matrix)
        R               % Control effort weight (2x2 matrix)
        S               % Control rate weight (2x2 matrix)
        Q_terminal      % Terminal state weight (3x3 matrix)
        
        % Constraints
        u_min           % Minimum control [v_min; ω_min]
        u_max           % Maximum control [v_max; ω_max]
        du_max          % Maximum control rate [Δv_max; Δω_max]
        
        % Optimization settings
        options         % fmincon options structure
        
        % Warm start (previous solution)
        u_prev          % Previous control sequence (2*N x 1)
        
        % Diagnostics
        solve_time      % Last solve time (seconds)
        iterations      % Number of iterations in last solve
        cost            % Cost value of last solution
        exitflag        % fmincon exit flag
    end
    
    methods
        function obj = nmpc_controller(N, Q, R, S, Q_terminal)
            % Constructor: Initialize NMPC controller
            %
            % INPUTS:
            %   N          - Prediction horizon (integer, typically 10-30)
            %   Q          - State tracking weight (3x3 matrix or scalar)
            %                If scalar: Q = q*eye(3)
            %   R          - Control effort weight (2x2 matrix or scalar)
            %                If scalar: R = r*eye(2)
            %   S          - Control rate weight (2x2 matrix or scalar)
            %                If scalar: S = s*eye(2)
            %   Q_terminal - Terminal cost weight (3x3 matrix or scalar)
            %                If scalar: Q_terminal = qf*eye(3)
            %
            % EXAMPLE:
            %   nmpc = nmpc_controller(20, 10, 0.1, 1.0, 100);
            %   This creates an NMPC with:
            %     - 20 step horizon
            %     - Strong state tracking (Q=10*I)
            %     - Small control penalty (R=0.1*I)
            %     - Moderate smoothness (S=1.0*I)
            %     - Very strong terminal cost (Qf=100*I)
            
            % Load robot parameters and model
            obj.params = nova_carter_params();
            obj.model = differential_drive_model();
            
            % Set MPC parameters
            obj.N = N;
            obj.dt = obj.params.dt;
            
            % Convert scalar weights to matrices if needed
            if isscalar(Q)
                obj.Q = Q * eye(3);
            else
                obj.Q = Q;
            end
            
            if isscalar(R)
                obj.R = R * eye(2);
            else
                obj.R = R;
            end
            
            if isscalar(S)
                obj.S = S * eye(2);
            else
                obj.S = S;
            end
            
            if isscalar(Q_terminal)
                obj.Q_terminal = Q_terminal * eye(3);
            else
                obj.Q_terminal = Q_terminal;
            end
            
            % Set control constraints from robot parameters
            obj.u_min = [obj.params.v_min; obj.params.omega_min];
            obj.u_max = [obj.params.v_max; obj.params.omega_max];
            
            % Set rate constraints (acceleration limits)
            % Note: These are limits per time step
            a_max = 2.0;        % Maximum linear acceleration (m/s²)
            alpha_max = 3.0;    % Maximum angular acceleration (rad/s²)
            
            obj.du_max = [a_max * obj.dt; alpha_max * obj.dt];
            
            % Configure optimization solver
            obj.options = optimoptions('fmincon', ...
                'Algorithm', 'sqp', ...              % Sequential Quadratic Programming
                'Display', 'none', ...               % Suppress output
                'MaxIterations', 100, ...            % Maximum iterations
                'MaxFunctionEvaluations', 2000, ... % Maximum function calls
                'ConstraintTolerance', 1e-6, ...    % Constraint satisfaction tolerance
                'OptimalityTolerance', 1e-6, ...    % Optimality tolerance
                'StepTolerance', 1e-10, ...         % Minimum step size
                'SpecifyObjectiveGradient', false, ... % Use numerical gradients
                'SpecifyConstraintGradient', false);   % Use numerical gradients
            
            % Initialize warm start (zeros = robot at rest)
            obj.u_prev = zeros(2*N, 1);
            
            % Initialize diagnostics
            obj.solve_time = 0;
            obj.iterations = 0;
            obj.cost = 0;
            obj.exitflag = 0;
        end
        
        function [u_opt, u_sequence, x_predicted] = solve(obj, x_current, x_ref_traj, u_last)
            % Solve NMPC optimization problem
            %
            % INPUTS:
            %   x_current   - Current state from EKF [x; y; θ] (3x1)
            %   x_ref_traj  - Reference trajectory (3 x N+1 matrix)
            %                 Columns are [x_ref; y_ref; θ_ref] at each step
            %   u_last      - Last applied control [v; ω] (2x1)
            %                 Used for rate constraint and smoothness
            %
            % OUTPUTS:
            %   u_opt       - Optimal control for current step [v; ω] (2x1)
            %   u_sequence  - Full optimal control sequence (2 x N)
            %   x_predicted - Predicted state trajectory (3 x N+1)
            %
            % ALGORITHM:
            %   1. Initialize with warm start (shift previous solution)
            %   2. Define cost function and constraints
            %   3. Call fmincon to solve NLP
            %   4. Extract optimal control
            %   5. Compute predicted trajectory for diagnostics
            
            tic;  % Start timing
            
            % Warm start: shift previous solution and repeat last control
            % This significantly speeds up convergence
            if ~isempty(obj.u_prev) && length(obj.u_prev) == 2*obj.N
                % Shift: [u_1, u_2, ..., u_{N-1}, u_{N-1}]
                u_init = [obj.u_prev(3:end); obj.u_prev(end-1:end)];
            else
                % Cold start: initialize with zeros
                u_init = zeros(2*obj.N, 1);
            end
            
            % Define cost function (anonymous function closure)
            cost_fn = @(u_vec) obj.cost_function(u_vec, x_current, x_ref_traj, u_last);
            
            % Define nonlinear constraints (dynamics are implicit in cost)
            % For differential drive, we don't need explicit dynamics constraints
            % because we use single shooting (states computed from controls)
            nonlcon = [];
            
            % Box constraints: lb ≤ u ≤ ub for each time step
            lb = repmat(obj.u_min, obj.N, 1);  % [u_min; u_min; ...; u_min]
            ub = repmat(obj.u_max, obj.N, 1);  % [u_max; u_max; ...; u_max]
            
            % Linear inequality constraints for rate limits
            % Represents: |u_k - u_{k-1}| ≤ du_max
            % Formulated as: -du_max ≤ u_k - u_{k-1} ≤ du_max
            [A_rate, b_rate] = obj.build_rate_constraints(u_last);
            
            % Solve nonlinear program
            [u_solution, cost_val, exitflag, output] = fmincon(...
                cost_fn, ...        % Objective function
                u_init, ...         % Initial guess
                A_rate, b_rate, ... % Linear inequality constraints (rate limits)
                [], [], ...         % No linear equality constraints
                lb, ub, ...         % Box constraints
                nonlcon, ...        % Nonlinear constraints (none)
                obj.options);       % Solver options
            
            % Store diagnostics
            obj.solve_time = toc;
            obj.cost = cost_val;
            obj.exitflag = exitflag;
            obj.iterations = output.iterations;
            if obj.iterations > 0 && mod(obj.iterations, 10) == 0
                obj.print_diagnostics();
            end
            
            % Warn if optimization failed
            if exitflag < 0
                warning('NMPC: Optimization failed (exitflag=%d, iter=%d)', exitflag, obj.iterations);
            % elseif exitflag == 0 && mod(obj.iterations, 10)
            % 
            %     fprintf('[NMPC] Optimization reached iteration limit (exitflag=0, iter=%d)\n', obj.iterations);
            end
                        
            % Extract results
            u_sequence = reshape(u_solution, 2, obj.N);  % Reshape to 2xN
            u_opt = u_sequence(:, 1);  % First control action (apply this!)
            
            % Store for next warm start
            obj.u_prev = u_solution;
            
            % Compute predicted trajectory (for visualization/debugging)
            x_predicted = obj.predict_trajectory(x_current, u_sequence);
            

        end
        
        function J = cost_function(obj, u_vec, x_current, x_ref_traj, u_last)
            % Compute MPC cost function
            %
            % INPUTS:
            %   u_vec      - Control sequence [u_0; u_1; ...; u_{N-1}] (2N x 1)
            %   x_current  - Initial state [x; y; θ] (3x1)
            %   x_ref_traj - Reference trajectory (3 x N+1)
            %   u_last     - Previous control input (2x1)
            %
            % OUTPUT:
            %   J          - Total cost (scalar)
            %
            % COST BREAKDOWN:
            %   Stage cost (k=0 to N-1):
            %     - State tracking: ||x_k - x_ref_k||²_Q
            %     - Control effort: ||u_k||²_R
            %     - Control smoothness: ||u_k - u_{k-1}||²_S
            %   Terminal cost:
            %     - ||x_N - x_ref_N||²_Qf
            
            % Reshape control vector to matrix
            u_sequence = reshape(u_vec, 2, obj.N);  % 2 x N
            
            % Predict trajectory from control sequence
            x_predicted = obj.predict_trajectory(x_current, u_sequence);
            
            % Initialize cost
            J = 0;
            
            % Stage costs (k = 0 to N-1)
            for k = 1:obj.N
                % State tracking error
                e_state = x_predicted(:,k) - x_ref_traj(:,k);
                
                % CRITICAL: Wrap angle error to [-π, π]
                % Without this, 359° and 1° would have huge error
                e_state(3) = obj.params.wrapToPi(e_state(3));
                
                % Control at this step
                u_k = u_sequence(:,k);
                
                % Control rate (difference from previous)
                if k == 1
                    % Compare to last applied control
                    du_k = u_k - u_last;
                else
                    % Compare to previous step in sequence
                    du_k = u_k - u_sequence(:,k-1);
                end
                
                % Accumulate stage cost
                % J_k = e^T Q e + u^T R u + du^T S du
                J = J + e_state' * obj.Q * e_state ...      % State tracking
                      + u_k' * obj.R * u_k ...              % Control effort
                      + du_k' * obj.S * du_k;               % Control smoothness
            end
            
            % Terminal cost (k = N)
            e_terminal = x_predicted(:,end) - x_ref_traj(:,end);
            e_terminal(3) = obj.params.wrapToPi(e_terminal(3));
            
            J = J + e_terminal' * obj.Q_terminal * e_terminal;
        end
        
        function x_traj = predict_trajectory(obj, x0, u_sequence)
            % Predict state trajectory from control sequence
            %
            % INPUTS:
            %   x0         - Initial state [x; y; θ] (3x1)
            %   u_sequence - Control sequence (2 x N)
            %
            % OUTPUT:
            %   x_traj     - Predicted states (3 x N+1)
            %                Columns: [x_0, x_1, ..., x_N]
            %
            % DYNAMICS:
            %   x_{k+1} = f(x_k, u_k) using robot's kinematic model
            
            N = size(u_sequence, 2);
            x_traj = zeros(3, N+1);
            x_traj(:,1) = x0;
            
            % Forward simulate using dynamics
            for k = 1:N
                u_k = u_sequence(:,k);
                % Use the validated dynamics model
                x_traj(:,k+1) = obj.model.dynamics_discrete(x_traj(:,k), u_k);
            end
        end
        
       function [A, b] = build_rate_constraints(obj, u_last)
            % Build linear inequality constraints for control rate limits
            %
            % FORMULATION:
            %   For k=0: -du_max ≤ u_0 - u_last ≤ du_max
            %   For k≥1: -du_max ≤ u_k - u_{k-1} ≤ du_max
            %
            % MATRIX FORM:
            %   A * u_vec ≤ b
            %   where u_vec = [v_0; ω_0; v_1; ω_1; ...; v_{N-1}; ω_{N-1}]
            %
            % CONSTRUCTION:
            %   Each control variable gets two inequality constraints:
            %     Upper bound:  u_k - u_{k-1} ≤ du_max
            %     Lower bound: -u_k + u_{k-1} ≤ du_max (i.e., u_k ≥ u_{k-1} - du_max)
            %
            % EXAMPLE for N=2:
            %   u_vec = [v_0; ω_0; v_1; ω_1]
            %   
            %   Constraints:
            %     v_0 - v_last ≤ dv_max    →  [1  0  0  0] * u_vec ≤ dv_max + v_last
            %    -v_0 + v_last ≤ dv_max    →  [-1 0  0  0] * u_vec ≤ dv_max - v_last
            %     ω_0 - ω_last ≤ dω_max    →  [0  1  0  0] * u_vec ≤ dω_max + ω_last
            %    -ω_0 + ω_last ≤ dω_max    →  [0 -1  0  0] * u_vec ≤ dω_max - ω_last
            %     v_1 - v_0 ≤ dv_max       →  [-1 0  1  0] * u_vec ≤ dv_max
            %    -v_1 + v_0 ≤ dv_max       →  [1  0 -1  0] * u_vec ≤ dv_max
            %     ω_1 - ω_0 ≤ dω_max       →  [0 -1  0  1] * u_vec ≤ dω_max
            %    -ω_1 + ω_0 ≤ dω_max       →  [0  1  0 -1] * u_vec ≤ dω_max
            
            n_controls = 2 * obj.N;  % Total decision variables
            
            % Each control variable has 2 constraints (upper and lower bound)
            % We have 2 control variables per time step (v and ω)
            % Total: 2 * 2 * N = 4N constraints
            n_constraints = 4 * obj.N;
            
            A = zeros(n_constraints, n_controls);
            b = zeros(n_constraints, 1);
            
            row = 1;
            
            % First time step: compare u_0 to u_last
            % Control vector at k=0 is [v_0; ω_0] at indices [1; 2]
            
            % Constraint: v_0 - v_last ≤ dv_max
            A(row, 1) = 1;
            b(row) = obj.du_max(1) + u_last(1);
            row = row + 1;
            
            % Constraint: -v_0 + v_last ≤ dv_max
            A(row, 1) = -1;
            b(row) = obj.du_max(1) - u_last(1);
            row = row + 1;
            
            % Constraint: ω_0 - ω_last ≤ dω_max
            A(row, 2) = 1;
            b(row) = obj.du_max(2) + u_last(2);
            row = row + 1;
            
            % Constraint: -ω_0 + ω_last ≤ dω_max
            A(row, 2) = -1;
            b(row) = obj.du_max(2) - u_last(2);
            row = row + 1;
            
            % Subsequent time steps: compare u_k to u_{k-1}
            for k = 2:obj.N
                % Indices for u_k = [v_k; ω_k]
                idx_v_curr = 2*(k-1) + 1;  % Index of v_k
                idx_w_curr = 2*(k-1) + 2;  %// Index of ω_k
                
                % Indices for u_{k-1} = [v_{k-1}; ω_{k-1}]
                idx_v_prev = 2*(k-2) + 1;  % Index of v_{k-1}
                idx_w_prev = 2*(k-2) + 2;  % Index of ω_{k-1}
                
                % Constraint: v_k - v_{k-1} ≤ dv_max
                A(row, idx_v_curr) = 1;
                A(row, idx_v_prev) = -1;
                b(row) = obj.du_max(1);
                row = row + 1;
                
                % Constraint: -v_k + v_{k-1} ≤ dv_max
                A(row, idx_v_curr) = -1;
                A(row, idx_v_prev) = 1;
                b(row) = obj.du_max(1);
                row = row + 1;
                
                % Constraint: ω_k - ω_{k-1} ≤ dω_max
                A(row, idx_w_curr) = 1;
                A(row, idx_w_prev) = -1;
                b(row) = obj.du_max(2);
                row = row + 1;
                
                %// Constraint: -ω_k + ω_{k-1} ≤ dω_max
                A(row, idx_w_curr) = -1;
                A(row, idx_w_prev) = 1;
                b(row) = obj.du_max(2);
                row = row + 1;
            end
        end
        
        function print_diagnostics(obj)
            % Print diagnostic information about last solve
            %
            % Useful for debugging and performance monitoring
            
            fprintf('NMPC Diagnostics:\n');
            fprintf('  Solve time: %.3f ms\n', obj.solve_time * 1000);
            fprintf('  Iterations: %d\n', obj.iterations);
            fprintf('  Cost: %.4f\n', obj.cost);
            fprintf('  Exit flag: %d ', obj.exitflag);
            
            switch obj.exitflag
                case 1
                    fprintf('(Success: First-order optimality)\n');
                case 2
                    fprintf('(Success: Step size below threshold)\n');
                case 0
                    fprintf('(Max iterations reached)\n');
                case -1
                    fprintf('(Stopped by output/plot function)\n');
                case -2
                    fprintf('(No feasible point found)\n');
                otherwise
                    fprintf('(Unknown)\n');
            end
        end
    end
end