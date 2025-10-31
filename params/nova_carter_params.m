%% nova_carter_params.m
% Nova Carter Robot Parameters and Constants
% Based on official specifications

classdef nova_carter_params
    properties (Constant)
        % Physical dimensions (from manual)
        wheel_radius = 0.140;      % m (280mm diameter wheels)
        track_width = 0.414;       % m (distance between wheel centers)
        half_track = 0.207;        % m (L/2)
        
        % Robot dimensions (for visualization/collision)
        length = 0.722;            % m
        width = 0.500;             % m
        height = 0.556;            % m
        
        % Velocity limits (from specs: ≥12 km/h = 3.33 m/s)
        v_max = 3.0;               % m/s (conservative)
        v_min = -0.5;              % m/s (slow reverse)
        omega_max = 2.0;           % rad/s (≈115 deg/s)
        omega_min = -2.0;          % rad/s
        
        % Acceleration limits (estimated, tune in sim)
        a_max = 2.0;               % m/s^2 (linear acceleration)
        alpha_max = 3.0;           % rad/s^2 (angular acceleration)
        
        % MPC parameters
        dt = 0.02;                 % s (50 Hz control rate)
        %N =40 ;                    % prediction horizon steps (0.5s)
        N =40 ;                   % prediction horizon steps (0.8s) - INCREASED
        
        % Cost function weights (to be tuned)
        Q_pos = 10.0;              % position error weight
        Q_theta = 5.0;             % heading error weight
        R_v = 0.1;                 % linear velocity effort
        R_omega = 0.1;             % angular velocity effort
        S_v = 0.5;                 % linear velocity rate penalty
        S_omega = 0.5;             % angular velocity rate penalty
        
        % Physical limits
        max_payload = 50;          % kg
        robot_mass = 49.6;         % kg (without payload)
    end
    
    methods (Static)
        function th_wrapped = wrapToPi(th)
            % Wrap angle to [-pi, pi]
            th_wrapped = mod(th + pi, 2*pi) - pi;
        end
        
        function err = angularError(th, th_ref)
            % Compute wrapped angular error
            err = nova_carter_params.wrapToPi(th - th_ref);
        end
        
        function [v_wheels, omega_wheels] = chassis2wheels(v, omega)
            % Convert chassis velocities to wheel velocities
            % Returns: [v_R, v_L] (right and left wheel speeds in m/s)
            r = nova_carter_params.wheel_radius;
            l = nova_carter_params.half_track;
            
            % Wheel angular velocities (rad/s)
            omega_R = (v + l*omega) / r;
            omega_L = (v - l*omega) / r;
            
            % Wheel linear velocities (m/s)
            v_wheels = [omega_R * r; omega_L * r];
            omega_wheels = [omega_R; omega_L];
        end
        
        function [v, omega] = wheels2chassis(omega_R, omega_L)
            % Convert wheel velocities to chassis velocities
            % Inputs: omega_R, omega_L (wheel angular velocities in rad/s)
            r = nova_carter_params.wheel_radius;
            l = nova_carter_params.half_track;
            
            v = r/2 * (omega_R + omega_L);
            omega = r/(2*l) * (omega_R - omega_L);
        end
    end
end