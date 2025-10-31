%% forward_kinematics.m
% Direct wheel command interface for Nova Carter
% 
% PURPOSE: 
%   Simulates robot motion from WHEEL VELOCITIES (not chassis velocities)
%   This matches what Isaac Sim sensors report and what ROS2 commands expect
%
% WHY NEEDED:
%   - Isaac Sim publishes /joint_states with wheel angular velocities
%   - ROS2 controllers command wheels directly
%   - This validates our kinematic equations match the simulator
%
% EQUATIONS:
%   Chassis velocities from wheel velocities:
%     v = (r/2)(φ̇_R + φ̇_L)       [average of wheels]
%     ω = (r/L)(φ̇_R - φ̇_L)        [difference creates rotation]
%   
%   Robot motion:
%     ẋ = v cos(θ)
%     ẏ = v sin(θ)
%     θ̇ = ω

classdef forward_kinematics
    properties
        params  % nova_carter_params object
    end
    
    methods
        function obj = forward_kinematics()
            obj.params = nova_carter_params;
        end
        
        function x_next = propagate_from_wheels(obj, x, phi_dot_L, phi_dot_R, dt)
            % Propagate robot state from wheel angular velocities
            %
            % INPUTS:
            %   x          - current state [x; y; theta] (meters, meters, radians)
            %   phi_dot_L  - left wheel angular velocity (rad/s)
            %   phi_dot_R  - right wheel angular velocity (rad/s)
            %   dt         - time step (seconds) [optional, default: params.dt]
            %
            % OUTPUT:
            %   x_next     - next state [x; y; theta]
            %
            % EXAMPLE:
            %   Both wheels at 5 rad/s → straight forward
            %   Right wheel faster → turn left
            %   Opposite signs → spin in place

            if nargin < 5
                dt = obj.params.dt;
            end

            % Robot physical parameters
            r = obj.params.wheel_radius;   % 0.14 m
            L = obj.params.track_width;    % 0.414 m

            % STEP 1: Convert wheel velocities to chassis velocities
            % This is the CRITICAL transformation that must match Isaac Sim
            function dx = dynamics(x_local)
                theta = x_local(3);
                v = (r/2) * (phi_dot_R + phi_dot_L);      % Linear velocity
                omega = (r/L) * (phi_dot_R - phi_dot_L);  % Angular velocity
                dx = [v * cos(theta);
                      v * sin(theta);
                      omega];
            end

            % STEP 2: RK4 integration
            k1 = dynamics(x);
            k2 = dynamics(x + 0.5 * dt * k1);
            k3 = dynamics(x + 0.5 * dt * k2);
            k4 = dynamics(x + dt * k3);

            x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

            % STEP 3: Wrap angle to [-π, π]
            x_next(3) = obj.params.wrapToPi(x_next(3));
        end
        
        function x_traj = simulate_wheel_profile(obj, x0, phi_dot_profile, T_sim)
            % Simulate entire trajectory from time-varying wheel speeds
            %
            % INPUTS:
            %   x0              - initial state [x; y; theta]
            %   phi_dot_profile - function handle: @(t) [phi_dot_L; phi_dot_R]
            %   T_sim           - simulation duration (seconds)
            %
            % OUTPUT:
            %   x_traj          - trajectory (3 x N_steps+1)
            %
            % EXAMPLE:
            %   profile = @(t) [5; 5];  % constant forward
            %   traj = fk.simulate_wheel_profile([0;0;0], profile, 10);

            dt = obj.params.dt;
            N_steps = round(T_sim / dt);

            x_traj = zeros(3, N_steps + 1);
            x_traj(:, 1) = x0;

            for k = 1:N_steps
                t = (k-1) * dt;
                phi_dot = phi_dot_profile(t);
                x_traj(:, k+1) = obj.propagate_from_wheels(...
                    x_traj(:, k), phi_dot(1), phi_dot(2), dt);
            end
        end
    end
end