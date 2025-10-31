%% differential_drive_model.m
% Kinematic model for differential drive robot (Option A)

classdef differential_drive_model
    properties
        params  % nova_carter_params object
    end
    
    methods
        function obj = differential_drive_model()
            obj.params = nova_carter_params;
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