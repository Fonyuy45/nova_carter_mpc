%% sensor_noise_simulator.m
% Simulates realistic sensor noise for testing EKF
%
% PURPOSE:
%   Generate noisy sensor readings that mimic real hardware
%   - Wheel encoders: quantization + Gaussian noise
%   - IMU gyroscope: bias + Gaussian noise + drift
%   - GPS: position measurement with realistic noise
%
% USAGE:
%   % Without GPS (backward compatible)
%   noise_sim = sensor_noise_simulator('low', 'low');
%   
%   % With GPS
%   noise_sim = sensor_noise_simulator('low', 'low', 'medium');
%
% WHY REALISTIC NOISE?
%   - Tests EKF robustness
%   - Tunes noise covariances (Q, R)
%   - Prepares for real hardware deployment
%
% VERSION HISTORY:
%   v1.0 - Encoder + IMU
%   v2.0 - Added GPS support (backward compatible)

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
        
        % GPS noise parameters (NEW)
        gps_sigma_x       % Std dev of x position noise (m)
        gps_sigma_y       % Std dev of y position noise (m)
        gps_enabled       % Flag: is GPS available?
        
        % Random number generator state (for repeatability)
        rng_state
    end
    
    methods
        function obj = sensor_noise_simulator(enc_noise_level, imu_noise_level, gps_noise_level)
            % Constructor: sets noise parameters based on level
            %
            % INPUTS:
            %   enc_noise_level - 'low', 'medium', or 'high'
            %   imu_noise_level - 'low', 'medium', or 'high'
            %   gps_noise_level - OPTIONAL: 'low', 'medium', 'high', or 'none'
            %                     If not provided, GPS is disabled (backward compatible)
            %
            % EXAMPLES:
            %   % Old style (no GPS)
            %   sim = sensor_noise_simulator('low', 'low');
            %
            %   % New style (with GPS)
            %   sim = sensor_noise_simulator('low', 'low', 'medium');
            
            obj.params = nova_carter_params();  % Load robot geometry
            
            % ================================================================
            % ENCODER NOISE PARAMETERS
            % ================================================================
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
                    error('Invalid encoder noise level. Use ''low'', ''medium'', or ''high''');
            end
            
            % ================================================================
            % // IMU NOISE PARAMETERS
            % ================================================================
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
                    error('Invalid IMU noise level. Use ''low'', ''medium'', or ''high''');
            end
            
            % ================================================================
            % GPS NOISE PARAMETERS (NEW - BACKWARD COMPATIBLE)
            % ================================================================
            if nargin < 3 || isempty(gps_noise_level)
                % GPS not specified - disable it (backward compatible)
                obj.gps_enabled = false;
                obj.gps_sigma_x = 0;
                obj.gps_sigma_y = 0;
            else
                obj.gps_enabled = true;
                
                switch lower(gps_noise_level)
                    case 'none'
                        % GPS disabled explicitly
                        obj.gps_enabled = false;
                        obj.gps_sigma_x = 0;
                        obj.gps_sigma_y = 0;
                        
                    case 'low'
                        % RTK GPS / DGPS quality
                        obj.gps_sigma_x = 0.5;   % 50 cm horizontal accuracy
                        obj.gps_sigma_y = 0.5;   % (CEP: ~60cm)
                        
                    case 'medium'
                        % Standard consumer GPS
                        obj.gps_sigma_x = 2.0;   % 2 m horizontal accuracy
                        obj.gps_sigma_y = 2.0;   % (CEP: ~2.4m)
                        
                    case 'high'
                        % Poor GPS (urban canyon, multipath)
                        obj.gps_sigma_x = 5.0;   % 5 m horizontal accuracy
                        obj.gps_sigma_y = 5.0;   % (CEP: ~6m)
                        
                    otherwise
                        error('Invalid GPS noise level. Use ''low'', ''medium'', ''high'', or ''none''');
                end
            end
            
            % Initialize RNG for reproducibility
            obj.rng_state = rng(42);  % Fixed seed
            
            % Print configuration summary
            if obj.gps_enabled
                fprintf('  Sensor noise initialized: Enc=%s, IMU=%s, GPS=%s\n', ...
                        enc_noise_level, imu_noise_level, gps_noise_level);
            else
                fprintf('  Sensor noise initialized: Enc=%s, IMU=%s (GPS disabled)\n', ...
                        enc_noise_level, imu_noise_level);
            end
        end
        
        % ====================================================================
        % ENCODER SENSOR
        % ====================================================================
        function z_enc = add_encoder_noise(obj, v_true, omega_true)
            % Simulate noisy encoder measurements
            %
            % INPUTS:
            %   v_true     - true linear velocity (m/s)
            %   omega_true - true angular velocity (rad/s)
            %
            % OUTPUT:
            %   z_enc      - noisy measurement [v_meas; omega_meas]
            %
            % NOISE MODEL:
            %   1. Add Gaussian noise to velocities
            %   2. Convert to wheel speeds
            %   3. Apply encoder quantization
            %   4. Convert back to body velocities
            
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
        
        % ====================================================================
        % IMU SENSOR
        % ====================================================================
        function z_imu = add_imu_noise(obj, omega_true, t)
            % Simulate noisy IMU gyroscope measurement
            %
            % INPUTS:
            %   omega_true - true angular velocity (rad/s)
            %   t          - current time (s)
            %
            % OUTPUT:
            %   z_imu      - noisy gyro reading (rad/s)
            %
            % NOISE MODEL:
            %   z = ω_true + bias(t) + noise
            %   bias(t) = bias_0 + drift_rate * t
            
            % Compute current bias with drift
            bias_current = obj.imu_bias_gyro + obj.imu_bias_drift * t;
            
            % Add bias and Gaussian noise
            z_imu = omega_true + bias_current + obj.imu_sigma_gyro * randn();
        end
        
        % ====================================================================
        % GPS SENSOR (NEW)
        % ====================================================================
        function z_gps = add_gps_noise(obj, x_true, y_true)
            % Simulate noisy GPS position measurement
            %
            % INPUTS:
            %   x_true - true x position (m)
            %   y_true - true y position (m)
            %
            % OUTPUT:
            %   z_gps - noisy measurement [x_meas; y_meas] (2×1)
            %
            % NOISE MODEL:
            %   - Independent Gaussian noise on x and y
            %   - Circular Error Probable (CEP) ≈ 1.2 × σ
            %   - No correlation between x and y errors (simplified)
            %
            % USAGE:
            %   z_gps = noise_sim.add_gps_noise(x_true(1), x_true(2));
            %
            % NOTE: If GPS is disabled, this function will error
            
            if ~obj.gps_enabled
                error('GPS sensor is not enabled. Initialize with gps_noise_level parameter.');
            end
            
            % Add independent Gaussian noise to each coordinate
            noise_x = obj.gps_sigma_x * randn();
            noise_y = obj.gps_sigma_y * randn();
            
            z_gps = [x_true + noise_x; 
                     y_true + noise_y];
        end
        
        % ====================================================================
        % QUERY METHODS
        % ====================================================================
        function flag = is_gps_enabled(obj)
            % Check if GPS sensor is available
            %
            % OUTPUT:
            %   flag - true if GPS is enabled, false otherwise
            flag = obj.gps_enabled;
        end
        
        function R_gps = get_gps_covariance(obj)
            % Get GPS measurement noise covariance matrix
            %
            % OUTPUT:
            %   R_gps - 2×2 covariance matrix for [x; y] measurement
            %
            % USAGE:
            %   R_gps = noise_sim.get_gps_covariance();
            %   ekf.update_gps(z_gps, R_gps);
            
            if ~obj.gps_enabled
                error('GPS sensor is not enabled');
            end
            
            R_gps = diag([obj.gps_sigma_x^2, obj.gps_sigma_y^2]);
        end
        
        % ====================================================================
        % UTILITY METHODS
        % ====================================================================
        function reset_rng(obj, seed)
            % Reset RNG for repeatable noise generation
            %
            % INPUT:
            %   seed - optional seed value (default: 42)
            if nargin < 2
                seed = 42;
            end
            obj.rng_state = rng(seed);
        end
        
        function print_config(obj)
            % Print current sensor configuration
            
            fprintf('\n========================================\n');
            fprintf('Sensor Noise Configuration\n');
            fprintf('========================================\n');
            
            fprintf('ENCODERS:\n');
            fprintf('  σ_v     = %.4f m/s (%.2f cm/s)\n', obj.enc_sigma_v, obj.enc_sigma_v*100);
            fprintf('  σ_ω     = %.4f rad/s (%.2f deg/s)\n', obj.enc_sigma_omega, rad2deg(obj.enc_sigma_omega));
            fprintf('  Quant   = %.4f rad (%.2f deg)\n', obj.enc_quantization, rad2deg(obj.enc_quantization));
            
            fprintf('\nIMU:\n');
            fprintf('  σ_gyro  = %.4f rad/s (%.2f deg/s)\n', obj.imu_sigma_gyro, rad2deg(obj.imu_sigma_gyro));
            fprintf('  Bias    = %.4f rad/s (%.2f deg/s)\n', obj.imu_bias_gyro, rad2deg(obj.imu_bias_gyro));
            fprintf('  Drift   = %.6f rad/s²\n', obj.imu_bias_drift);
            
            if obj.gps_enabled
                fprintf('\nGPS:\n');
                fprintf('  σ_x     = %.2f m\n', obj.gps_sigma_x);
                fprintf('  σ_y     = %.2f m\n', obj.gps_sigma_y);
                fprintf('  CEP     = %.2f m (50%% error < this)\n', 1.2 * mean([obj.gps_sigma_x, obj.gps_sigma_y]));
            else
                fprintf('\nGPS: DISABLED\n');
            end
            
            fprintf('========================================\n\n');
        end
    end
end