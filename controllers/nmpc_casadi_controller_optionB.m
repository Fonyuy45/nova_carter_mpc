%% nmpc_casadi_controller_optionB.m
%
% PURPOSE:
%   NMPC controller with first-order actuator dynamics (Option B)
%   Models realistic motor lag for smoother, more realizable control commands
%
% STATE: x = [x, y, θ, v, ω]^T  (5D)
%   - x, y:   Position (m)
%   - θ:      Heading (rad, unwrapped for optimization)
%   - v:      Linear velocity (m/s) - STATE, not control!
%   - ω:      Angular velocity (rad/s) - STATE, not control!
%
% CONTROL: u = [v_cmd, ω_cmd]^T  (2D)
%   - v_cmd:  Commanded linear velocity
%   - ω_cmd:  Commanded angular velocity
%
% DYNAMICS:
%   Kinematics:  ẋ = v·cos(θ), ẏ = v·sin(θ), θ̇ = ω
%   Actuators:   v̇ = (v_cmd - v)/τ_v,  ω̇ = (ω_cmd - ω)/τ_ω
%
% WHY OPTION B?
%   - Accounts for motor inertia and controller lag
%   - Produces smoother, more realizable commands
%   - Better matches real hardware behavior
%   - Reduces mechanical stress and wheel slip
%
% AUTHOR: Nova Carter NMPC Team
% DATE: November 2025

classdef nmpc_casadi_controller_optionB
    properties
        % MPC parameters
        N               % Prediction horizon
        dt              % Time step (s)
        
        % Cost function weights
        Q               % State tracking weight (5x5)
        R               % Control effort weight (2x2)
        S               % Control rate weight (2x2)
        Qf              % Terminal cost weight (5x5)
        
        % Constraints
        u_min           % Control lower bounds [v_min; ω_min]
        u_max           % Control upper bounds [v_max; ω_max]
        du_max          % Rate limits [Δv_max; Δω_max]
        
        % Actuator dynamics parameters
        tau_v           % Linear velocity time constant (s)
        tau_omega       % Angular velocity time constant (s)
        alpha_v         % Discrete filter coefficient for v
        alpha_omega     % Discrete filter coefficient for ω
        
        % CasADi solver objects
        solver          % IPOPT solver (built once in constructor)
        lb_U            % Control lower bounds (repeated N times)
        ub_U            % Control upper bounds (repeated N times)
        lb_g            % Constraint lower bounds
        ub_g            % Constraint upper bounds
        
        % Warm start
        u_prev          % Previous solution (2N×1)
        
        % Diagnostics
        solve_times     % History of solve times
        n_solves        % Total number of solves
        last_status     % Last solver status
    end
    
    methods
        function obj = nmpc_casadi_controller_optionB(N, Q, R, S, Qf, dt, ...
                u_min, u_max, du_max, tau_v, tau_omega)
            % Constructor: Build NMPC with actuator dynamics
            %
            % INPUTS:
            %   N         - Prediction horizon (e.g., 20-50 steps)
            %   Q         - State tracking weight (5x5 matrix or scalar)
            %               If scalar: Q = q*eye(5)
            %               Suggested: diag([q_x, q_y, q_θ, q_v, q_ω])
            %               Example: diag([40, 5, 30, 0.1, 0.1])
            %   R         - Control effort weight (2x2 matrix or scalar)
            %               Penalizes large control commands
            %               Example: diag([0.1, 0.5])
            %   S         - Control rate weight (2x2 matrix or scalar)
            %               Penalizes rapid command changes
            %               Example: diag([1.0, 2.0])
            %   Qf        - Terminal cost weight (5x5 matrix or scalar)
            %               Strong penalty on final state error
            %               Example: 10*Q
            %   dt        - Time step (s), typically 0.01
            %   u_min     - Control lower bounds [v_min; ω_min]
            %               Example: [0.0; -π/2]
            %   u_max     - Control upper bounds [v_max; ω_max]
            %               Example: [1.5; π/2]
            %   du_max    - Rate limits per time step [Δv_max; Δω_max]
            %               Example: [a_max*dt; α_max*dt] = [0.025; 0.03]
            %   tau_v     - Linear velocity time constant (s)
            %               Typical: 0.15-0.25s (DC motor response)
            %               Smaller = faster response, larger = more lag
            %   tau_omega - Angular velocity time constant (s)
            %               Typical: 0.10-0.20s (often faster than linear)
            %
            % EXAMPLE:
            %   N = 50;
            %   Q = diag([40, 5, 30, 0.1, 0.1]);  % [x, y, θ, v, ω]
            %   R = diag([0.1, 0.5]);
            %   S = diag([1.0, 2.0]);
            %   Qf = 10 * Q;
            %   dt = 0.01;
            %   u_min = [0; -pi/2];
            %   u_max = [1.5; pi/2];
            %   du_max = [0.025; 0.03];
            %   tau_v = 0.2;
            %   tau_omega = 0.15;
            %   
            %   nmpc = nmpc_casadi_controller_optionB(N, Q, R, S, Qf, dt, ...
            %       u_min, u_max, du_max, tau_v, tau_omega);
            
            import casadi.*
            
            fprintf('\n========================================\n');
            fprintf('NMPC Controller - Option B\n');
            fprintf('========================================\n');
            
            % Store parameters
            obj.N = N;
            obj.dt = dt;
            obj.tau_v = tau_v;
            obj.tau_omega = tau_omega;
            obj.u_min = u_min;
            obj.u_max = u_max;
            obj.du_max = du_max;
            
            % Precompute discrete actuator filter coefficients
            % From continuous: v̇ = (v_cmd - v)/τ
            % To discrete: v[k+1] = α·v_cmd + (1-α)·v[k]
            % Where: α = dt/(τ + dt)
            obj.alpha_v = dt / (tau_v + dt);
            obj.alpha_omega = dt / (tau_omega + dt);
            
            fprintf('  Actuator dynamics:\n');
            fprintf('    τ_v = %.3f s (α_v = %.3f)\n', tau_v, obj.alpha_v);
            fprintf('    τ_ω = %.3f s (α_ω = %.3f)\n', tau_omega, obj.alpha_omega);
            fprintf('    Time to 63%% response: v=%.0fms, ω=%.0fms\n', ...
                    tau_v*1000, tau_omega*1000);
            
            % Convert scalar weights to matrices (5D state!)
            if isscalar(Q)
                obj.Q = Q * eye(5);
                fprintf('  State weights: Q = %.1f*I (5×5)\n', Q);
            else
                if size(Q,1) ~= 5 || size(Q,2) ~= 5
                    error('Q must be 5×5 for Option B (state is 5D)');
                end
                obj.Q = Q;
                fprintf('  State weights: Q = diag([%.1f, %.1f, %.1f, %.2f, %.2f])\n', ...
                        diag(Q));
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
            
            if isscalar(Qf)
                obj.Qf = Qf * eye(5);
            else
                if size(Qf,1) ~= 5 || size(Qf,2) ~= 5
                    error('Qf must be 5×5 for Option B');
                end
                obj.Qf = Qf;
            end
            
            fprintf('  Horizon: N = %d steps (%.2fs lookahead)\n', N, N*dt);
            fprintf('\n  Building symbolic optimization problem...\n');
            
            % =================================================================
            % SYMBOLIC PROBLEM DEFINITION
            % =================================================================
            
            % Decision variables: control sequence
            U = SX.sym('U', 2, obj.N);  % [v_cmd, ω_cmd] at each step
            U_vec = reshape(U, 2 * obj.N, 1);
            
            % Parameters: runtime data
            P_x0 = SX.sym('P_x0', 5, 1);              % Current state [x;y;θ;v;ω]
            P_ref = SX.sym('P_ref', 5, obj.N + 1);    % Reference trajectory (5D)
            P_ulast = SX.sym('P_ulast', 2, 1);        % Last applied control
            
            % Pack all parameters into single vector
            P = [P_x0; reshape(P_ref, 5 * (obj.N + 1), 1); P_ulast];
            
            % Initialize
            x_traj = SX.zeros(5, obj.N + 1);  % Predicted state trajectory
            x_traj(:,1) = P_x0;               % Initial condition
            J = 0;                            % Cost accumulator
            g = [];                           % Constraint vector
            
            % =================================================================
            % BUILD COST AND CONSTRAINTS
            % =================================================================
            
            for k = 1:obj.N
                % Extract current predicted state and control
                x_k = x_traj(:,k);
                u_k = U(:,k);
                x_ref_k = P_ref(:,k);
                
                % --- STAGE COST ---
                
                % State tracking error
                e = x_k - x_ref_k;
                e(3) = wrapToPiCasadi(e(3));  % Wrap heading error to [-π,π]
                
                % Control rate (smoothness penalty)
                if k == 1
                    du = u_k - P_ulast;  % Compare to last applied control
                else
                    du = u_k - U(:,k-1); % Compare to previous command in sequence
                end
                
                % Accumulate cost
                % J = Σ[e'Qe + u'Ru + du'Sdu]
                J = J + e' * obj.Q * e ...       % State tracking
                      + u_k' * obj.R * u_k ...   % Control effort
                      + du' * obj.S * du;        % Control smoothness
                
                % --- RATE CONSTRAINTS ---
                % Enforce: -du_max ≤ du ≤ du_max
                g = [g; du];
                
                % --- DYNAMICS: KINEMATICS + ACTUATOR LAG ---
                
                % Extract state components
                x_pos = x_k(1);      % Position x
                y_pos = x_k(2);      % Position y
                theta = x_k(3);      % Heading
                v = x_k(4);          % Current velocity (state)
                omega = x_k(5);      % Current angular velocity (state)
                
                % Extract control commands
                v_cmd = u_k(1);      % Commanded velocity
                omega_cmd = u_k(2);  % Commanded angular velocity
                
                % Kinematic update (using CURRENT velocities v, ω)
                % This is the KEY difference from Option A!
                % We predict position based on current velocities, not commands
                x_next_pos = x_pos + v * cos(theta) * obj.dt;
                y_next_pos = y_pos + v * sin(theta) * obj.dt;
                theta_next = theta + omega * obj.dt;
                
                % Actuator dynamics (first-order lag)
                % Discrete low-pass filter: x[k+1] = α·x_target + (1-α)·x[k]
                % This models: τ·ẋ = x_target - x
                v_next = obj.alpha_v * v_cmd + (1 - obj.alpha_v) * v;
                omega_next = obj.alpha_omega * omega_cmd + (1 - obj.alpha_omega) * omega;
                
                % Assemble next state
                x_traj(:,k+1) = [x_next_pos; y_next_pos; theta_next; v_next; omega_next];
            end
            
            % --- TERMINAL COST ---
            e_terminal = x_traj(:,end) - P_ref(:,end);
            e_terminal(3) = wrapToPiCasadi(e_terminal(3));
            J = J + e_terminal' * obj.Qf * e_terminal;
            
            fprintf('    ✓ Cost function built\n');
            
            % =================================================================
            % CREATE NLP SOLVER
            % =================================================================
            
            % Define nonlinear program
            nlp = struct('x', U_vec, 'f', J, 'g', g, 'p', P);
            
            % Solver options (tuned for Option B)
            opts = struct();
            
            % IPOPT options
            opts.ipopt.print_level = 0;                    % Silent
            opts.ipopt.suppress_all_output = 'yes';
            opts.ipopt.tol = 1e-6;                         % Convergence tolerance
            opts.ipopt.constr_viol_tol = 1e-6;            % Constraint satisfaction
            opts.ipopt.acceptable_tol = 1e-5;             % Acceptable tolerance
            opts.ipopt.acceptable_constr_viol_tol = 1e-5;
            opts.ipopt.max_iter = 50;                      % Max iterations
            opts.ipopt.warm_start_init_point = 'yes';     % Enable warm start
            opts.ipopt.mu_strategy = 'adaptive';          % Adaptive barrier
            opts.ipopt.adaptive_mu_globalization = 'kkt-error';
            opts.ipopt.nlp_scaling_method = 'gradient-based';
            
            % CasADi options
            opts.print_time = false;
            opts.verbose = false;
            opts.expand = true;  % Expand NLP (can improve performance)
            
            fprintf('    ✓ Creating IPOPT solver...\n');
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            fprintf('    ✓ Solver ready\n');
            
            % =================================================================
            % STORE BOUNDS
            % =================================================================
            
            % Control bounds (repeated for each time step)
            obj.lb_U = repmat(obj.u_min, obj.N, 1);
            obj.ub_U = repmat(obj.u_max, obj.N, 1);
            
            % Rate constraint bounds: -du_max ≤ du ≤ du_max
            % Constraint vector g has dimension 2N (2 per time step)
            obj.lb_g = repmat(-obj.du_max, obj.N, 1);
            obj.ub_g = repmat(obj.du_max, obj.N, 1);
            
            % Initialize warm start
            obj.u_prev = zeros(2 * obj.N, 1);
            
            % Initialize diagnostics
            obj.solve_times = [];
            obj.n_solves = 0;
            obj.last_status = '';
            
            fprintf('\n========================================\n');
            fprintf('✓ NMPC Controller Ready (Option B)\n');
            fprintf('  State dimension: 5D [x, y, θ, v, ω]\n');
            fprintf('  Control dimension: 2D [v_cmd, ω_cmd]\n');
            fprintf('  Prediction horizon: %d steps\n', N);
            fprintf('  Decision variables: %d\n', 2*N);
            fprintf('  Constraints: %d\n', 2*N);
            fprintf('========================================\n\n');
        end
        
        function [u_opt, x_pred] = solve(obj, x0, x_ref_traj, u_last)
            % Solve NMPC optimization problem
            %
            % INPUTS:
            %   x0         - Current state [x; y; θ; v; ω] (5×1)
            %   x_ref_traj - Reference trajectory (5 × N+1)
            %                Each column: [x_ref; y_ref; θ_ref; v_ref; ω_ref]
            %   u_last     - Last applied control [v; ω] (2×1)
            %
            % OUTPUTS:
            %   u_opt  - Optimal control command [v_cmd; ω_cmd] (2×1)
            %            THIS IS THE COMMAND, NOT THE ACTUAL VELOCITY!
            %            Actual velocity will lag behind due to actuator dynamics
            %   x_pred - Predicted state trajectory (5 × N+1)
            %            For visualization and debugging
            %
            % NOTES:
            %   - State must be 5D (includes velocity states v, ω)
            %   - Reference must be 5D
            %   - Solver exploits warm starting for speed
            
            % ===== INPUT VALIDATION =====
            if length(x0) ~= 5
                error('State x0 must be 5D [x;y;θ;v;ω], got %dD', length(x0));
            end
            
            if size(x_ref_traj, 1) ~= 5
                error('Reference must be 5D, got %dD', size(x_ref_traj, 1));
            end
            
            if size(x_ref_traj, 2) < obj.N + 1
                error('Reference must have at least N+1=%d points, got %d', ...
                      obj.N+1, size(x_ref_traj, 2));
            end
            
            tic;  % Start timing
            
            % ===== PACK PARAMETER VECTOR =====
            % Must match order defined in constructor
            p_vec = [x0; ...
                     reshape(x_ref_traj(:, 1:obj.N+1), 5 * (obj.N + 1), 1); ...
                     u_last];
            
            % ===== WARM START =====
            % Shift previous solution: [u_1, u_2, ..., u_{N-1}, u_{N-1}]
            % This gives a good initial guess for current solve
            if ~isempty(obj.u_prev) && length(obj.u_prev) == 2*obj.N
                u_init = [obj.u_prev(3:end); obj.u_prev(end-1:end)];
            else
                % Cold start: initialize with zero commands
                u_init = zeros(2 * obj.N, 1);
            end
            
            % ===== SOLVE OPTIMIZATION =====
            try
                sol = obj.solver(...
                    'x0',  u_init, ...      % Initial guess
                    'p',   p_vec, ...       % Parameters
                    'lbx', obj.lb_U, ...    % Lower bounds on U
                    'ubx', obj.ub_U, ...    % Upper bounds on U
                    'lbg', obj.lb_g, ...    % Lower bounds on g (rate constraints)
                    'ubg', obj.ub_g);       % Upper bounds on g
                
                obj.last_status = 'success';
                
            catch ME
                warning('NMPC: Solver error: %s', ME.message);
                obj.last_status = 'error';
                % Return previous control as fallback
                if ~isempty(obj.u_prev)
                    u_opt = obj.u_prev(1:2);
                else
                    u_opt = [0; 0];  % Emergency stop
                end
                x_pred = [];
                return;
            end
            
            solve_time = toc;
            
            % ===== STORE DIAGNOSTICS =====
            obj.solve_times = [obj.solve_times, solve_time];
            obj.n_solves = obj.n_solves + 1;
            
            % ===== EXTRACT RESULTS =====
            u_solution = full(sol.x);
            
            % Store for next warm start
            obj.u_prev = u_solution;
            
            % Extract first control (receding horizon principle)
            u_opt = u_solution(1:2);
            
            % ===== PREDICT TRAJECTORY =====
            % Forward simulate using optimal control sequence
            x_pred = obj.predict_trajectory(x0, u_solution);
        end
        
        function x_pred = predict_trajectory(obj, x0, u_solution)
            % Predict state trajectory from control sequence
            %
            % INPUTS:
            %   x0         - Initial state (5×1)
            %   u_solution - Control sequence (2N×1)
            %
            % OUTPUT:
            %   x_pred     - Predicted trajectory (5 × N+1)
            %
            % Uses same dynamics as optimization (kinematics + actuator lag)
            
            u_seq = reshape(u_solution, 2, obj.N);
            x_pred = zeros(5, obj.N + 1);
            x_pred(:,1) = x0;
            
            for k = 1:obj.N
                x_k = x_pred(:,k);
                u_cmd_k = u_seq(:,k);
                
                % Extract states
                x_pos = x_k(1);
                y_pos = x_k(2);
                theta = x_k(3);
                v = x_k(4);
                omega = x_k(5);
                
                % Kinematics (using current velocities)
                x_next = x_pos + v * cos(theta) * obj.dt;
                y_next = y_pos + v * sin(theta) * obj.dt;
                theta_next = theta + omega * obj.dt;
                
                % Actuator dynamics (first-order lag)
                v_next = obj.alpha_v * u_cmd_k(1) + (1 - obj.alpha_v) * v;
                omega_next = obj.alpha_omega * u_cmd_k(2) + (1 - obj.alpha_omega) * omega;
                
                x_pred(:,k+1) = [x_next; y_next; theta_next; v_next; omega_next];
            end
        end
        
        function print_diagnostics(obj)
            % Print detailed diagnostic information
            
            if isempty(obj.solve_times)
                fprintf('No solves yet\n');
                return;
            end
            
            fprintf('\n========================================\n');
            fprintf('NMPC Diagnostics (Option B)\n');
            fprintf('========================================\n');
            fprintf('Configuration:\n');
            fprintf('  Horizon: N = %d\n', obj.N);
            fprintf('  Time step: dt = %.3f s\n', obj.dt);
            fprintf('  Actuator lag: τ_v=%.3fs, τ_ω=%.3fs\n', ...
                    obj.tau_v, obj.tau_omega);
            
            fprintf('\nPerformance:\n');
            fprintf('  Total solves:     %d\n', obj.n_solves);
            fprintf('  Avg solve time:   %.2f ms\n', mean(obj.solve_times)*1000);
            fprintf('  Max solve time:   %.2f ms\n', max(obj.solve_times)*1000);
            fprintf('  Min solve time:   %.2f ms\n', min(obj.solve_times)*1000);
            fprintf('  Std solve time:   %.2f ms\n', std(obj.solve_times)*1000);
            fprintf('  Last status:      %s\n', obj.last_status);
            
            % Real-time feasibility analysis
            fprintf('\nReal-Time Feasibility:\n');
            
            rates = [100, 50, 30, 20];  % Hz
            deadlines = 1000 ./ rates;  % ms
            
            for i = 1:length(rates)
                deadline_s = deadlines(i) / 1000;
                miss_rate = sum(obj.solve_times > deadline_s) / obj.n_solves * 100;
                
                fprintf('  %3d Hz (%2.0f ms): %5.1f%% misses', ...
                        rates(i), deadlines(i), miss_rate);
                
                if miss_rate < 1
                    fprintf(' ✓✓ (Excellent)\n');
                elseif miss_rate < 5
                    fprintf(' ✓  (Good)\n');
                elseif miss_rate < 20
                    fprintf(' ~  (Acceptable)\n');
                else
                    fprintf(' ✗  (Too slow)\n');
                end
            end
            
            fprintf('========================================\n\n');
        end
        
        function reset_diagnostics(obj)
            % Reset diagnostic counters
            obj.solve_times = [];
            obj.n_solves = 0;
            obj.last_status = '';
        end
        
        function info = get_solver_info(obj)
            % Return solver information as struct
            info = struct();
            info.N = obj.N;
            info.dt = obj.dt;
            info.tau_v = obj.tau_v;
            info.tau_omega = obj.tau_omega;
            info.n_solves = obj.n_solves;
            if ~isempty(obj.solve_times)
                info.avg_solve_time = mean(obj.solve_times);
                info.max_solve_time = max(obj.solve_times);
                info.min_solve_time = min(obj.solve_times);
            else
                info.avg_solve_time = 0;
                info.max_solve_time = 0;
                info.min_solve_time = 0;
            end
            info.last_status = obj.last_status;
        end
    end
end

% =============================================================================
% HELPER FUNCTIONS
% =============================================================================

function angle = wrapToPiCasadi(angle)
    % CasADi-compatible angle wrapping to [-π, π]
    % Symbolic equivalent of MATLAB's wrapToPi function
    %
    % FORMULA: angle_wrapped = angle - 2π⌊(angle + π)/(2π)⌋
    import casadi.*
    angle = angle - 2*pi * floor((angle + pi) / (2*pi));
end