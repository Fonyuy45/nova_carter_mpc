%% nmpc_casadi_controller.m
%
% PURPOSE:
%   Optimal trajectory tracking controller using CasADi.
%   Solves a finite-horizon optimal control problem at each time step
%   using a pre-built nonlinear solver (IPOPT).
%
% KEY WORKFLOW (IMPORTANT):
%   1. CONSTRUCTOR: We define the entire optimization problem symbolically
%      (decision variables 'U', parameters 'P'). We build the symbolic
%      cost 'J' and constraints 'g' *once*. We create the 'nlpsol'
%      solver object *once* and store it.
%   2. SOLVE: This function is called at every time step. It is now very
%      lightweight. It just packs the new numerical data (x0, x_ref_traj)
%      into the parameter vector 'p' and calls the pre-built solver.
%      This is the key to real-time performance.
%
classdef nmpc_casadi_controller
    properties
        % MPC parameters
        N               % Prediction horizon
        dt              % Time step
        Q               % State tracking weight
        R               % Control effort weight
        S               % Control rate weight
        Qf              % Terminal cost weight
        u_min           % Control lower bounds [v_min; omega_min]
        u_max           % Control upper bounds [v_max; omega_max]
        du_max          % Rate limits [dv_max; domega_max]
        
        % CasADi objects
        solver          % CasADi nlpsol solver object (created once)
        lb_U            % Lower bounds for decision variables (U)
        ub_U            % Upper bounds for decision variables (U)
        lb_g            % Lower bounds for constraints (g)
        ub_g            % Upper bounds for constraints (g)
        
        % Warm start
        u_prev          % Previous control sequence solution (2*N x 1)
    end
    
    methods
        function obj = nmpc_casadi_controller(N, Q, R, S, Qf, dt, u_min, u_max, du_max)
            % Constructor: Build the NMPC problem and solver *once*
            
            % Import CasADi package
            import casadi.*
            
            % Store parameters
            obj.N = N;
            obj.dt = dt;
            obj.Q = Q;
            obj.R = R;
            obj.S = S;
            obj.Qf = Qf;
            obj.u_min = u_min;
            obj.u_max = u_max;
            obj.du_max = du_max;
            
            % -----------------------------------------------------------------
            % 1. DEFINE SYMBOLIC VARIABLES
            % -----------------------------------------------------------------
            
            % Decision variables (the control sequence we are solving for)
            % U = [u_0, u_1, ..., u_{N-1}]
            U = SX.sym('U', 2, obj.N);
            U_vec = reshape(U, 2 * obj.N, 1); % Flatten to a vector
            
            % Parameters (all data that changes at runtime)
            % P = [x_current; x_ref_0; ...; x_ref_N; u_last]
            P_x0 = SX.sym('P_x0', 3, 1);                  % Current state
            P_ref = SX.sym('P_ref', 3, obj.N + 1);      % Reference trajectory
            P_ulast = SX.sym('P_ulast', 2, 1);            % Last applied control
            
            % Pack all parameters into a single vector 'P'
            P = [P_x0; reshape(P_ref, 3 * (obj.N + 1), 1); P_ulast];
            
            % -----------------------------------------------------------------
            % 2. BUILD SYMBOLIC GRAPH (COST & CONSTRAINTS)
            % -----------------------------------------------------------------
            
            % Initialize
            x_traj = SX.zeros(3, obj.N + 1);  % Symbolic state trajectory
            x_traj(:,1) = P_x0;             % Initial state is a parameter
            J = 0;                            % Initialize cost
            g = [];                           % Initialize constraint vector
            
            % Build the cost and constraints at each step
            for k = 1:obj.N
                % Get current state, control, and reference
                x_k = x_traj(:,k);
                u_k = U(:,k);
                x_ref_k = P_ref(:,k);
                
                % --- Stage Cost ---
                % State tracking error
                e = x_k - x_ref_k;
                e(3) = wrapToPiCasadi(e(3)); % Handle angle wrapping
                
                % Control rate (smoothness)
                if k == 1
                    du = u_k - P_ulast; % Compare to P_ulast parameter
                else
                    du = u_k - U(:,k-1);% Compare to previous decision
                end
                
                % Accumulate stage cost
                J = J + e' * obj.Q * e + u_k' * obj.R * u_k + du' * obj.S * du;
                
                % --- Hard Rate Constraints ---
                % We add du to the constraint vector 'g'
                % We will later define bounds: -du_max <= du <= du_max
                g = [g; du];
                
                % --- Dynamics (System Model) ---
                % Predict next state using discrete-time (Euler) model
                x_next = [x_k(1) + u_k(1) * cos(x_k(3)) * obj.dt;
                          x_k(2) + u_k(1) * sin(x_k(3)) * obj.dt;
                          x_k(3) + u_k(2) * obj.dt];
                
                x_traj(:,k+1) = x_next;
            end
            
            % --- Terminal Cost ---
            e_terminal = x_traj(:,end) - P_ref(:,end);
            e_terminal(3) = wrapToPiCasadi(e_terminal(3));
            J = J + e_terminal' * obj.Qf * e_terminal;
            
            % -----------------------------------------------------------------
            % 3. CREATE THE NLP SOLVER
            % -----------------------------------------------------------------
            
            % Define the NLP (Nonlinear Program)
            % 'x' are the decision variables (U_vec)
            % 'f' is the cost function (J)
            % 'g' is the constraint vector (all the 'du's)
            % 'p' are the parameters (P)
            nlp = struct('x', U_vec, 'f', J, 'g', g, 'p', P);
            
            % --- CORRECTED CODE HERE ---
            % Solver options. We must create a nested struct for 'ipopt'.
            opts = struct();
            opts.ipopt = struct('print_level', 0, 'suppress_all_output', 'yes');
            opts.print_time = false;
            % --- END CORRECTION ---

            % Create the solver function *once* and store it in the object
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % -----------------------------------------------------------------
            % 4. STORE BOUNDS
            % -----------------------------------------------------------------
            
            % Store bounds for decision variables (U)
            obj.lb_U = repmat(obj.u_min, obj.N, 1);
            obj.ub_U = repmat(obj.u_max, obj.N, 1);
            
            % Store bounds for constraints (g)
            % We have N rate constraints: -du_max <= du_k <= du_max
            % The constraint vector 'g' is [du_0; du_1; ...; du_{N-1}]
            % Each 'du_k' is [dv_k; domega_k], so g is 2N x 1
            obj.lb_g = repmat(-obj.du_max, obj.N, 1);
            obj.ub_g = repmat(obj.du_max, obj.N, 1);
            
            % Initialize warm start
            obj.u_prev = zeros(2 * obj.N, 1);
        end
        
        function [u_opt, x_pred] = solve(obj, x0, x_ref_traj, u_last)
            % Solve NMPC optimization problem using the pre-built solver
            
            % -----------------------------------------------------------------
            % 1. PACK PARAMETER VECTOR
            % -----------------------------------------------------------------
            % Pack all runtime data into the single parameter vector 'p_vec'
            % in the *exact same order* as defined in the constructor.
            p_vec = [x0; reshape(x_ref_traj, 3 * (obj.N + 1), 1); u_last];
            
            % -----------------------------------------------------------------
            % 2. GET INITIAL GUESS (WARM START)
            % -----------------------------------------------------------------
            % Shift the previous solution to use as an initial guess
            % This dramatically speeds up convergence.
            if ~isempty(obj.u_prev)
                u_init = [obj.u_prev(3:end); obj.u_prev(end-1:end)];
            else
                u_init = zeros(2 * obj.N, 1);
            end
            
            % -----------------------------------------------------------------
            % 3. CALL THE SOLVER
            % -----------------------------------------------------------------
            % Call the pre-built solver object
            sol = obj.solver(...
                'x0',  u_init, ...    % Initial guess for U
                'p',   p_vec, ...     % Parameter values
                'lbx', obj.lb_U, ...  % Lower bounds for U
                'ubx', obj.ub_U, ...  % Upper bounds for U
                'lbg', obj.lb_g, ...  % Lower bounds for g
                'ubg', obj.ub_g  ...  % Upper bounds for g
            );
            
            % -----------------------------------------------------------------
            % 4. EXTRACT RESULTS
            % -----------------------------------------------------------------
            
            % Get the full optimal control sequence
            u_solution = full(sol.x);
            
            % Store for next warm start
            obj.u_prev = u_solution;
            
            % Get the first control action to apply
            u_opt = u_solution(1:2);
            
            % -----------------------------------------------------------------
            % 5. (OPTIONAL) PREDICT TRAJECTORY FOR DIAGNOSTICS
            % -----------------------------------------------------------------
            % This is a quick numerical simulation, not symbolic
            u_seq = reshape(u_solution, 2, obj.N);
            x_pred = zeros(3, obj.N + 1);
            x_pred(:,1) = x0;
            for k = 1:obj.N
                u_k = u_seq(:,k);
                x_k = x_pred(:,k);
                % Use the same dynamics as in the constructor
                x_next = [x_k(1) + u_k(1) * cos(x_k(3)) * obj.dt;
                          x_k(2) + u_k(1) * sin(x_k(3)) * obj.dt;
                          x_k(3) + u_k(2) * obj.dt];
                x_pred(:,k+1) = x_next;
            end
        end
    end
end

% Helper function MUST be outside the classdef
function angle = wrapToPiCasadi(angle)
    % CasADi-compatible angle wrapping to [-pi, pi]
    % This is the symbolic equivalent of the 'wrapToPi' function
    import casadi.*
    angle = angle - 2*pi * floor((angle + pi) / (2*pi));
end