classdef nmpc_casadi_controller
    properties
        N               % Prediction horizon
        dt              % Time step
        Q               % State tracking weight
        R               % Control effort weight
        S               % Control rate weight
        Qf              % Terminal cost weight
        u_min           % Control lower bounds [v_min; omega_min]
        u_max           % Control upper bounds [v_max; omega_max]
        du_max          % Rate limits [dv_max; domega_max]
        solver          % CasADi solver object
        u_prev          % Previous control sequence (warm start)
    end

    methods
        function obj = nmpc_casadi_controller(N, Q, R, S, Qf, dt, u_min, u_max, du_max)
            % Constructor: Initialize NMPC controller using CasADi
            import casadi.*

            obj.N = N;
            obj.dt = dt;
            obj.Q = Q;
            obj.R = R;
            obj.S = S;
            obj.Qf = Qf;
            obj.u_min = u_min;
            obj.u_max = u_max;
            obj.du_max = du_max;
            obj.u_prev = zeros(2 * N, 1);  % Initialize warm start

            % Define symbolic variables
            x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
            v = SX.sym('v'); omega = SX.sym('omega');
            X = [x; y; theta]; U = [v; omega];

            % Dynamics function
            x_next = [x + v * cos(theta) * dt;
                      y + v * sin(theta) * dt;
                      theta + omega * dt];
            obj.solver = [];  % Will be built dynamically in solve()
        end

        function [u_opt, x_pred] = solve(obj, x0, x_ref_traj, u_last)
            import casadi.*

            % Define decision variables: control sequence U = [u_0, ..., u_{N-1}]
            U = SX.sym('U', 2, obj.N);

            % Initialize trajectory and cost
            x_traj = SX.zeros(3, obj.N + 1);
            x_traj(:,1) = x0;
            J = 0;

            for k = 1:obj.N
                u_k = U(:,k);
                x_k = x_traj(:,k);
                x_ref_k = x_ref_traj(:,k);

                % Predict next state
                x_next = [x_k(1) + u_k(1) * cos(x_k(3)) * obj.dt;
                          x_k(2) + u_k(1) * sin(x_k(3)) * obj.dt;
                          x_k(3) + u_k(2) * obj.dt];
                x_traj(:,k+1) = x_next;

                % Tracking error
                e = x_k - x_ref_k;
                e(3) = wrapToPi(e(3));

                % Control rate
                if k == 1
                    du = u_k - u_last;
                else
                    du = u_k - U(:,k-1);
                end

                % Stage cost
                J = J + e' * obj.Q * e + u_k' * obj.R * u_k + du' * obj.S * du;
            end

            % Terminal cost
            e_terminal = x_traj(:,end) - x_ref_traj(:,end);
            e_terminal(3) = wrapToPi(e_terminal(3));
            J = J + e_terminal' * obj.Qf * e_terminal;

            % Flatten decision variables
            U_vec = reshape(U, 2 * obj.N, 1);

            % Bounds
            lb = repmat(obj.u_min, obj.N, 1);
            ub = repmat(obj.u_max, obj.N, 1);

            % Build NLP
            nlp = struct('x', U_vec, 'f', J);
            opts = struct('ipopt.print_level', 0, 'print_time', false);
            solver = nlpsol('solver', 'ipopt', nlp, opts);

            % Initial guess
            if ~isempty(obj.u_prev)
                u_init = [obj.u_prev(3:end); obj.u_prev(end-1:end)];
            else
                u_init = zeros(2 * obj.N, 1);
            end

            % Solve
            sol = solver('x0', u_init, 'lbx', lb, 'ubx', ub);
            u_solution = full(sol.x);
            obj.u_prev = u_solution;

            % Extract optimal control
            u_opt = u_solution(1:2);
            u_seq = reshape(u_solution, 2, obj.N);

            % Predict trajectory for diagnostics
            x_pred = zeros(3, obj.N + 1);
            x_pred(:,1) = x0;
            for k = 1:obj.N
                u_k = u_seq(:,k);
                x_k = x_pred(:,k);
                x_next = [x_k(1) + u_k(1) * cos(x_k(3)) * obj.dt;
                          x_k(2) + u_k(1) * sin(x_k(3)) * obj.dt;
                          x_k(3) + u_k(2) * obj.dt];
                x_pred(:,k+1) = x_next;
            end
        end
    end
end

function angle = wrapToPi(angle)
    % Wrap angle to [-pi, pi]
    angle = mod(angle + pi, 2*pi) - pi;
end