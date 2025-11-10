%% differential_drive_model.m
% Kinematic model for differential drive robot (Option A)

classdef differential_drive_model
    properties
        params  % nova_carter_params object
        tau_v       % Linear velocity time constant (s)
        tau_omega   % Angular velocity time constant (s)
    end
    
    methods
        function obj = differential_drive_model()
            obj.params = nova_carter_params;
            obj.tau_v = 0.2;        % 200ms for linear velocity (conservative)
            obj.tau_omega = 0.15;   % 150ms for angular velocity (faster steering)
        end
        
        function x_next = dynamics_continuous(obj, x, u)
            % Continuous-time dynamics: dx/dt = f(x,u)
            % State: x = [x; y; theta]
            % Input: u = [v; omega]
            
            x_pos = x(1);
            y_pos = x(2);
            theta = x(3);
            
            v = u(1);
            omega = u(2);
            
            % Kinematic equations
            x_dot = v * cos(theta);
            y_dot = v * sin(theta);
            theta_dot = omega;
            
            x_next = [x_dot; y_dot; theta_dot];
        end
        

        function x_next = dynamics_discrete(obj, x, u)
            % Discrete-time dynamics using forward Euler
            % x_{k+1} = x_k + f(x_k, u_k) * dt
            
            dt = obj.params.dt;
            x_dot = obj.dynamics_continuous(x, u);

            x_next = x + x_dot * dt;
            
            % Wrap theta to [-pi, pi]
            x_next(3) = obj.params.wrapToPi(x_next(3));
        end
        
        function x_next = dynamics_discrete_rk4(obj, x, u)
            % Discrete-time dynamics using RK4 (more accurate)
            dt = obj.params.dt;
            
            k1 = obj.dynamics_continuous(x, u);
            k2 = obj.dynamics_continuous(x + dt/2*k1, u);
            k3 = obj.dynamics_continuous(x + dt/2*k2, u);
            k4 = obj.dynamics_continuous(x + dt*k3, u);
            
            x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
            x_next(3) = obj.params.wrapToPi(x_next(3));
        end

        
        function x_next = dynamics_discrete_with_actuators(obj, x, u_cmd)
            % Discrete-time dynamics with first-order actuator lag (Option B)
            %
            % INPUTS:
            %   x     - Current state [x; y; θ; v; ω] (5x1)
            %   u_cmd - Control commands [v_cmd; ω_cmd] (2x1)
            %
            % OUTPUT:
            %   x_next - Next state (5x1)
            %
            % DYNAMICS:
            %   Position/heading: Standard kinematic model (using current v, ω)
            %   Velocities: First-order lag → τ·v̇ = (v_cmd - v)
            %
            % DISCRETE FORM:
            %   v_{k+1} = α·v_cmd + (1-α)·v_k,  where α = Δt/(τ + Δt)
            
            % ===== Input Validation =====
            if length(x) ~= 5
                error('dynamics_discrete_with_actuators: State must be 5D [x; y; θ; v; ω]');
            end
            
            if length(u_cmd) ~= 2
                error('dynamics_discrete_with_actuators: Control must be 2D [v_cmd; ω_cmd]');
            end
            
            % Check if actuator time constants are set
            if isempty(obj.tau_v) || isempty(obj.tau_omega)
                error(['dynamics_discrete_with_actuators: Actuator time constants not set!\n' ...
                       'Set model.tau_v and model.tau_omega before calling this method.']);
            end
            
            % ===== Extract States =====
            x_pos = x(1);
            y_pos = x(2);
            theta = x(3);
            v = x(4);
            omega = x(5);
            
            % ===== Extract Commands =====
            v_cmd = u_cmd(1);
            omega_cmd = u_cmd(2);
            
            % ===== Time Step =====
            dt = obj.params.dt;
            
            % ===== Kinematic Update =====
            % CRITICAL: Use CURRENT velocities (v, ω), NOT commands!
            % This is the key difference from Option A
            x_next_pos = x_pos + v * cos(theta) * dt;
            y_next_pos = y_pos + v * sin(theta) * dt;
            theta_next = theta + omega * dt;
            
            % ===== Actuator Dynamics (First-Order Filter) =====
            % Exponential smoothing: x_new = α·x_target + (1-α)·x_current
            % Transfer function: H(s) = 1 / (τs + 1)
            % Discrete equivalent: H(z) with α = Δt / (τ + Δt)
            
            alpha_v = dt / (obj.tau_v + dt);
            alpha_omega = dt / (obj.tau_omega + dt);
            
            v_next = alpha_v * v_cmd + (1 - alpha_v) * v;
            omega_next = alpha_omega * omega_cmd + (1 - alpha_omega) * omega;
            
            % ===== Assemble Next State =====
            x_next = [x_next_pos; y_next_pos; theta_next; v_next; omega_next];
            
            % Note: We don't wrap theta_next here because Option B typically
            % works with unwrapped angles for smoother optimization.
            % Wrapping is done in the controller or plotting functions.
        end
        
        function [A, B] = linearize(obj, x, u)
            % Linearize dynamics around operating point (x, u)
            % dx/dt = A*x + B*u (approximately)
            
            theta = x(3);
            v = u(1);
            
            % Jacobian w.r.t. state
            A = [0, 0, -v*sin(theta);
                 0, 0,  v*cos(theta);
                 0, 0,  0];
            
            % Jacobian w.r.t. input
            B = [cos(theta), 0;
                 sin(theta), 0;
                 0,          1];
        end
        

        function [phi_dot_L, phi_dot_R] = convert_to_wheel_speeds(obj, u_cmd)
            v = u_cmd(1);
            omega = u_cmd(2);
            r = obj.params.wheel_radius;
            L = obj.params.track_width;
    
            phi_dot_R = (v + omega * L/2) / r;
            phi_dot_L = (v - omega * L/2) / r;
        end

        
        function valid = checkConstraints(obj, x, u, u_prev)
            % Check if state and input satisfy constraints
            
            v = u(1);
            omega = u(2);
            
            % Velocity constraints
            v_ok = (v >= obj.params.v_min) && (v <= obj.params.v_max);
            omega_ok = (omega >= obj.params.omega_min) && ...
                       (omega <= obj.params.omega_max);
            
            % Acceleration constraints (if u_prev provided)
            if nargin > 3
                dv = (v - u_prev(1)) / obj.params.dt;
                domega = (omega - u_prev(2)) / obj.params.dt;
                
                a_ok = abs(dv) <= obj.params.a_max;
                alpha_ok = abs(domega) <= obj.params.alpha_max;
                
                valid = v_ok && omega_ok && a_ok && alpha_ok;
            else
                valid = v_ok && omega_ok;
            end
        end
    end
end