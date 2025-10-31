%% sensor_noise_simulator.m
% Simulates realistic sensor noise for testing EKF
%
% PURPOSE:
%   Generate noisy sensor readings that mimic real hardware
%   - Wheel encoders: quantization + Gaussian noise
%   - IMU gyroscope: bias + Gaussian noise + drift
%
% WHY REALISTIC NOISE?
%   - Tests EKF robustness
%   - Tunes noise covariances (Q, R)
%   - Prepares for real hardware deployment

classdef sensor_noise_simulator
    properties
        params  % Robot physical parameters (wheel radius, track width)
        
        % Encoder noise parameters
        enc_sigma_v       % Std dev of linear velocity noise (m/s)
        enc_sigma_omega   % Std dev of angular velocity noise (rad/s)
        enc_quantization  % Encoder resolution (rad/tick)
        
        % IMU noise parameters
        imu_sigma_gyro    % Std dev of gyro noise (rad/s)
        imu_bias_gyro     % Constant bias in gyro (rad/s)
        imu_bias_drift    % Drift rate of bias over time (rad/s²)
        
        % Random number generator state (for repeatability)
        rng_state
    end
    
    methods
        function obj = sensor_noise_simulator(enc_noise_level, imu_noise_level)
            % Constructor: sets noise parameters based on level
            % INPUTS:
            %   enc_noise_level - 'low', 'medium', or 'high'
            %   imu_noise_level - 'low', 'medium', or 'high'
            
            obj.params = nova_carter_params();  % Load robot geometry
            
            % Set encoder noise parameters
            switch lower(enc_noise_level)
                case 'low'
                    obj.enc_sigma_v = 0.01;       % 1 cm/s
                    obj.enc_sigma_omega = 0.005;  % ~0.3°/s
                    obj.enc_quantization = 0.001; % ~0.06°
                case 'medium'
                    obj.enc_sigma_v = 0.05;       % 5 cm/s
                    obj.enc_sigma_omega = 0.02;   % ~1.1°/s
                    obj.enc_quantization = 0.001; % ~0.3°
                case 'high'
                    obj.enc_sigma_v = 0.1;        % 10 cm/s
                    obj.enc_sigma_omega = 0.05;   % ~2.9°/s
                    obj.enc_quantization = 0.01;  % ~0.6°
                otherwise
                    error('Invalid encoder noise level');
            end
            
            % Set IMU noise parameters
            switch lower(imu_noise_level)
                case 'low'
                    obj.imu_sigma_gyro = 0.005;   % ~0.3°/s
                    obj.imu_bias_gyro = 0.01;     % ~0.6°/s bias
                    obj.imu_bias_drift = 0.0001;  % Slow drift
                case 'medium'
                    obj.imu_sigma_gyro = 0.02;    % ~1.1°/s
                    obj.imu_bias_gyro = 0.05;     % ~2.9°/s bias
                    obj.imu_bias_drift = 0.0002;  % Moderate drift
                case 'high'
                    obj.imu_sigma_gyro = 0.05;    % ~2.9°/s
                    obj.imu_bias_gyro = 0.1;      % ~5.7°/s bias
                    obj.imu_bias_drift = 0.001;   % Fast drift
                otherwise
                    error('Invalid IMU noise level');
            end
            
            % Initialize RNG for reproducibility
            obj.rng_state = rng(42);  % Fixed seed
        end
        
        function z_enc = add_encoder_noise(obj, v_true, omega_true)
            % Simulate noisy encoder measurements
            % INPUTS:
            %   v_true     - true linear velocity (m/s)
            %   omega_true - true angular velocity (rad/s)
            % OUTPUT:
            %   z_enc      - noisy measurement [v_meas; omega_meas]
            
            % Add Gaussian noise to chassis velocities
            v_noisy = v_true + obj.enc_sigma_v * randn();
            omega_noisy = omega_true + obj.enc_sigma_omega * randn();
            
            % Convert to wheel angular velocities
            wheel_v_R = (v_noisy + omega_noisy * obj.params.track_width / 2) / obj.params.wheel_radius;
            wheel_v_L = (v_noisy - omega_noisy * obj.params.track_width / 2) / obj.params.wheel_radius;
            
            % Apply quantization (round to encoder resolution)
            wheel_v_R = round(wheel_v_R / obj.enc_quantization) * obj.enc_quantization;
            wheel_v_L = round(wheel_v_L / obj.enc_quantization) * obj.enc_quantization;
            
            % Convert back to noisy chassis velocities
            v_meas = obj.params.wheel_radius * (wheel_v_R + wheel_v_L) / 2;
            omega_meas = obj.params.wheel_radius * (wheel_v_R - wheel_v_L) / obj.params.track_width;
            
            z_enc = [v_meas; omega_meas];
        end
        
        function z_imu = add_imu_noise(obj, omega_true, t)
            % Simulate noisy IMU gyroscope measurement
            % INPUTS:
            %   omega_true - true angular velocity (rad/s)
            %   t          - current time (s)
            % OUTPUT:
            %   z_imu      - noisy gyro reading (rad/s)
            
            % Compute current bias with drift
            bias_current = obj.imu_bias_gyro + obj.imu_bias_drift * t;
            
            % Add bias and Gaussian noise
            z_imu = omega_true + bias_current + obj.imu_sigma_gyro * randn();
        end
        
        function reset_rng(obj, seed)
            % Reset RNG for repeatable noise generation
            % INPUT:
            %   seed - optional seed value (default: 42)
            if nargin < 2
                seed = 42;
            end
            obj.rng_state = rng(seed);
        end
    end
end