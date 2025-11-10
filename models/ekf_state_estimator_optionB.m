classdef ekf_state_estimator_optionB
    % EKF_STATE_ESTIMATOR_OPTIONB
    %
    % Extended Kalman Filter for a differential-drive / unicycle robot
    % with FIRST-ORDER actuator dynamics on v and omega.
    %
    % State vector (6x1):
    %   x(1) = x       [m]     - global x position
    %   x(2) = y       [m]     - global y position
    %   x(3) = theta   [rad]   - heading (UNWRAPPED)
    %   x(4) = v       [m/s]   - ACTUAL linear velocity
    %   x(5) = omega   [rad/s] - ACTUAL angular velocity
    %   x(6) = b       [rad/s] - gyro bias (slowly varying / constant)
    %
    % Inputs to predict():
    %   u = [v_cmd; omega_cmd]   commanded velocities
    %
    % Motion model (discrete, Option B):
    %   x_{k+1}     = x_k + v_k cos(theta_k) dt
    %   y_{k+1}     = y_k + v_k sin(theta_k) dt
    %   theta_{k+1} = theta_k + omega_k dt
    %   v_{k+1}     = alpha_v * v_cmd + (1 - alpha_v) * v_k
    %   omega_{k+1} = alpha_w * omega_cmd + (1 - alpha_w) * omega_k
    %   b_{k+1}     = b_k
    %
    %   where: alpha_v = dt / (tau_v + dt)
    %          alpha_w = dt / (tau_w + dt)
    %
    % Measurements:
    %   encoders:  z_enc = [v_meas; omega_meas]
    %   imu:       z_imu = omega_meas + b
    %   gps:       z_gps = [x_meas; y_meas]  (NEW in v2.1)
    %
    % VERSION HISTORY:
    %   v2.0 - Corrected actuator dynamics (exponential filter)
    %   v2.1 - Added GPS measurement update
    %
    % ---------------------------------------------------------------------
    % Author: Nova Carter Team
    % Date: November 2025
    % Version: 2.1 (Option B + GPS)
    % ---------------------------------------------------------------------

    properties
        % robot / timing
        params          % struct from nova_carter_params()
        dt              % sampling time [s]

        % actuator time constants
        tau_v           % linear velocity time constant [s]
        tau_w           % angular velocity time constant [s]

        % filter state
        x_hat           % current state estimate (6x1)
        P               % covariance (6x6)

        % noise
        Q               % process noise (6x6)
        R_enc           % encoder noise (2x2)
        R_imu           % imu noise (1x1)
        R_gps           % gps noise (2x2) - NEW

        % diagnostics
        innovation      % last innovation vector
        S               % last innovation covariance
        K               % last Kalman gain
        
        % gps statistics (NEW)
        gps_update_count  % number of GPS updates
        gps_last_update   % time of last GPS update
    end

    methods
        function obj = ekf_state_estimator_optionB(x0, P0, Q, R_enc, R_imu, R_gps)
            % Constructor: initialize EKF with initial state/covariances
            %
            % INPUTS:
            %   x0    - Initial state [x; y; θ; v; ω; b] (6x1)
            %   P0    - Initial covariance (6x6)
            %   Q     - Process noise (6x6 or scalar)
            %   R_enc - Encoder noise (2x2 or scalar)
            %   R_imu - IMU noise (1x1 or scalar)
            %   R_gps - GPS noise (2x2 or scalar) - OPTIONAL
            %
            % EXAMPLES:
            %   % Without GPS (backward compatible)
            %   ekf = ekf_state_estimator_optionB(x0, P0, Q, R_enc, R_imu);
            %
            %   % With GPS
            %   ekf = ekf_state_estimator_optionB(x0, P0, Q, R_enc, R_imu, R_gps);

            % 1) get params (dt, and maybe tau_v, tau_w)
            obj.params = nova_carter_params();
            obj.dt     = obj.params.dt;

            % 2) actuator time constants (try to read from params, else defaults)
            if isfield(obj.params, 'tau_v')
                obj.tau_v = obj.params.tau_v;
            else
                obj.tau_v = 0.2;   % 200 ms (conservative default)
            end

            if isfield(obj.params, 'tau_w')
                obj.tau_w = obj.params.tau_w;
            else
                obj.tau_w = 0.15;   % 150 ms (faster steering)
            end

            % 3) initial state and covariance
            obj.x_hat = x0;     % [x; y; theta; v; omega; b]
            obj.P     = P0;

            % 4) process noise (CORRECTED: properly scaled by units!)
            if isscalar(Q)
                Q_diag = [ Q,          % x position (m²)
                           Q,          % y position (m²)
                           Q * 0.01,   % heading (rad²)
                           Q,          % linear velocity (m²/s²)
                           Q * 0.01,   % angular velocity (rad²/s²)
                           Q * 0.001]; % bias (rad²/s²)
                obj.Q = diag(Q_diag);
            else
                if size(Q,1) ~= 6 || size(Q,2) ~= 6
                    error('Process noise Q must be 6x6 or scalar');
                end
                obj.Q = Q;
            end

            % 5) encoder noise
            if isscalar(R_enc)
                obj.R_enc = R_enc * eye(2);
            else
                if size(R_enc,1) ~= 2 || size(R_enc,2) ~= 2
                    error('Encoder noise R_enc must be 2x2 or scalar');
                end
                obj.R_enc = R_enc;
            end

            % 6) imu noise
            if isscalar(R_imu)
                obj.R_imu = R_imu;
            else
                error('IMU noise R_imu must be scalar (1x1)');
            end

            % 7) GPS noise (NEW - backward compatible)
            if nargin < 6 || isempty(R_gps)
                % No GPS specified - use default (won't be used unless update_gps called)
                obj.R_gps = diag([2.0^2, 2.0^2]);  % Default: 2m accuracy
            else
                if isscalar(R_gps)
                    obj.R_gps = R_gps * eye(2);
                else
                    if size(R_gps,1) ~= 2 || size(R_gps,2) ~= 2
                        error('GPS noise R_gps must be 2x2 or scalar');
                    end
                    obj.R_gps = R_gps;
                end
            end

            % 8) diagnostics init
            obj.innovation = [];
            obj.S          = [];
            obj.K          = [];
            obj.gps_update_count = 0;
            obj.gps_last_update  = 0;
            
            fprintf('  EKF initialized: tau_v=%.3fs, tau_w=%.3fs\n', ...
                    obj.tau_v, obj.tau_w);
        end

        % =================================================================
        % PREDICTION STEP
        % =================================================================
        function obj = predict(obj, u_cmd)
            % PREDICT  EKF prediction step with 1st-order actuator dynamics
            %
            % INPUTS:
            %   u_cmd = [v_cmd; omega_cmd]  (2x1)

            dt    = obj.dt;
            tau_v = obj.tau_v;
            tau_w = obj.tau_w;

            % Validate input
            if nargin < 2 || isempty(u_cmd)
                error('predict() requires control input u_cmd = [v_cmd; omega_cmd]');
            end
            
            if length(u_cmd) ~= 2
                error('u_cmd must be 2D [v_cmd; omega_cmd], got %dD', length(u_cmd));
            end

            % Unpack state
            x  = obj.x_hat;
            px = x(1);
            py = x(2);
            th = x(3);
            v  = x(4);
            w  = x(5);
            b  = x(6);

            % Unpack commands
            v_cmd = u_cmd(1);
            w_cmd = u_cmd(2);

            % Exponential filter coefficients
            alpha_v = dt / (tau_v + dt);
            alpha_w = dt / (tau_w + dt);

            % State prediction
            px_next = px + v * cos(th) * dt;
            py_next = py + v * sin(th) * dt;
            th_next = th + w * dt;
            v_next = alpha_v * v_cmd + (1 - alpha_v) * v;
            w_next = alpha_w * w_cmd + (1 - alpha_w) * w;
            b_next = b;

            obj.x_hat = [px_next; py_next; th_next; v_next; w_next; b_next];

            % Jacobian
            F = [ 1, 0, -v*sin(th)*dt,  cos(th)*dt,         0,                0;
                  0, 1,  v*cos(th)*dt,  sin(th)*dt,         0,                0;
                  0, 0,  1,             0,                  dt,               0;
                  0, 0,  0,             1 - alpha_v,        0,                0;
                  0, 0,  0,             0,           1 - alpha_w,             0;
                  0, 0,  0,             0,                  0,                1];

            % Covariance prediction
            obj.P = F * obj.P * F' + obj.Q;
        end

        % =================================================================
        % UPDATE STEPS (MEASUREMENT CORRECTIONS)
        % =================================================================
        
        function obj = update_encoders(obj, z_enc)
            % UPDATE_ENCODERS  EKF correction with wheel encoders
            %
            % INPUTS:
            %   z_enc = [v_meas; omega_meas]  (2x1)

            H = [0, 0, 0, 1, 0, 0;
                 0, 0, 0, 0, 1, 0];

            z_pred = H * obj.x_hat;
            y      = z_enc - z_pred;
            S = H * obj.P * H' + obj.R_enc;
            K = obj.P * H' / S;

            obj.x_hat = obj.x_hat + K * y;

            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_enc * K';

            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
        end

        function obj = update_imu(obj, z_imu)
            % UPDATE_IMU  EKF correction with gyro
            %
            % INPUTS:
            %   z_imu - scalar gyro measurement (rad/s)

            H = [0, 0, 0, 0, 1, 1];

            z_pred = H * obj.x_hat;
            y      = z_imu - z_pred;
            S = H * obj.P * H' + obj.R_imu;
            K = obj.P * H' / S;

            obj.x_hat = obj.x_hat + K * y;

            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_imu * K';

            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
        end

        function obj = update_encoders_and_imu(obj, z_enc, z_imu)
            % UPDATE_ENCODERS_AND_IMU  EKF correction with encoders + IMU
            %
            % INPUTS:
            %   z_enc = [v_meas; omega_meas]  (2x1)
            %   z_imu - scalar gyro measurement

            z = [z_enc; z_imu];

            H = [0, 0, 0, 1, 0, 0;
                 0, 0, 0, 0, 1, 0;
                 0, 0, 0, 0, 1, 1];

            R = blkdiag(obj.R_enc, obj.R_imu);

            z_pred = H * obj.x_hat;
            y      = z - z_pred;
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;

            obj.x_hat = obj.x_hat + K * y;

            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * R * K';

            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
        end

        % =================================================================
        % GPS UPDATE (NEW)
        % =================================================================
        function obj = update_gps(obj, z_gps, R_gps_override, t)
            % UPDATE_GPS  EKF correction with GPS position measurement
            %
            % INPUTS:
            %   z_gps          - [x_meas; y_meas] (2×1)
            %   R_gps_override - OPTIONAL: override noise covariance (2×2)
            %   t              - OPTIONAL: current time (for diagnostics)
            %
            % USAGE:
            %   % Use default R_gps from constructor
            %   ekf = ekf.update_gps(z_gps);
            %
            %   % Override with custom noise
            %   R_custom = diag([1.0^2, 1.0^2]);
            %   ekf = ekf.update_gps(z_gps, R_custom);
            %
            %   % With time stamp
            %   ekf = ekf.update_gps(z_gps, [], k*dt);
            %
            % MEASUREMENT MODEL:
            %   z_gps = [x; y] + noise
            %   GPS directly measures position (no heading!)
            
            % Validate input
            if nargin < 2 || isempty(z_gps)
                error('update_gps() requires z_gps = [x_meas; y_meas]');
            end
            
            if length(z_gps) ~= 2
                error('z_gps must be 2D [x; y], got %dD', length(z_gps));
            end
            
            % Use override covariance if provided
            if nargin >= 3 && ~isempty(R_gps_override)
                R_gps = R_gps_override;
                if isscalar(R_gps)
                    R_gps = R_gps * eye(2);
                end
            else
                R_gps = obj.R_gps;
            end
            
            % Update time stamp if provided
            if nargin >= 4 && ~isempty(t)
                obj.gps_last_update = t;
            end

            % Measurement matrix (GPS measures x and y only)
            H = [1, 0, 0, 0, 0, 0;   % x position
                 0, 1, 0, 0, 0, 0];  % y position

            % Innovation
            z_pred = H * obj.x_hat;
            y      = z_gps - z_pred;
            
            % Innovation covariance
            S = H * obj.P * H' + R_gps;
            
            % Kalman gain
            K = obj.P * H' / S;

            % State update
            obj.x_hat = obj.x_hat + K * y;

            % Covariance update (Joseph form for numerical stability)
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * R_gps * K';

            % Store diagnostics
            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
            obj.gps_update_count = obj.gps_update_count + 1;
        end

        % =================================================================
        % // INTERFACE METHODS
        % =================================================================
        
        function x_for_nmpc = get_state_for_controller(obj)
            % GET_STATE_FOR_CONTROLLER  Return 5D state for NMPC
            %
            % OUTPUT:
            %   x_for_nmpc = [x; y; θ; v; ω]  (5x1)
            
            x_for_nmpc = obj.x_hat(1:5);
        end

        function b_est = get_gyro_bias(obj)
            % GET_GYRO_BIAS  Return estimated gyro bias
            %
            % OUTPUT:
            %   b_est - estimated bias (rad/s)
            
            b_est = obj.x_hat(6);
        end

        function [x, y, theta, v, omega, b] = get_state(obj)
            % GET_STATE  Return full (unwrapped) state
            
            x     = obj.x_hat(1);
            y     = obj.x_hat(2);
            theta = obj.x_hat(3);
            v     = obj.x_hat(4);
            omega = obj.x_hat(5);
            b     = obj.x_hat(6);
        end

        function [x, y, theta_wrapped, v, omega, b] = get_state_wrapped(obj)
            % GET_STATE_WRAPPED  Return state with heading wrapped to [-π, π]
            
            x              = obj.x_hat(1);
            y              = obj.x_hat(2);
            theta_wrapped  = wrapToPi(obj.x_hat(3));
            v              = obj.x_hat(4);
            omega          = obj.x_hat(5);
            b              = obj.x_hat(6);
        end

        function cov_trace = get_uncertainty(obj)
            % GET_UNCERTAINTY  Return trace of covariance
            cov_trace = trace(obj.P);
        end

        function pos_cov = get_position_covariance(obj)
            % GET_POSITION_COVARIANCE  Return 2x2 covariance for (x,y)
            pos_cov = obj.P(1:2, 1:2);
        end
        
        function vel_cov = get_velocity_covariance(obj)
            % GET_VELOCITY_COVARIANCE  Return 2x2 covariance for (v,ω)
            vel_cov = obj.P(4:5, 4:5);
        end
        
        function count = get_gps_update_count(obj)
            % GET_GPS_UPDATE_COUNT  Return number of GPS updates (NEW)
            count = obj.gps_update_count;
        end
        
        function t = get_gps_last_update_time(obj)
            % GET_GPS_LAST_UPDATE_TIME  Return time of last GPS update (NEW)
            t = obj.gps_last_update;
        end

        % =================================================================
        % DIAGNOSTICS
        % =================================================================
        
        function print_diagnostics(obj)
            % PRINT_DIAGNOSTICS  Display current filter state and statistics
            
            fprintf('\n========================================\n');
            fprintf('EKF State Estimator (Option B + GPS)\n');
            fprintf('========================================\n');
            
            [x, y, theta, v, omega, b] = obj.get_state();
            
            fprintf('State Estimate:\n');
            fprintf('  Position:  (%.3f, %.3f) m\n', x, y);
            fprintf('  Heading:   %.2f° (%.3f rad)\n', rad2deg(theta), theta);
            fprintf('  Linear v:  %.3f m/s\n', v);
            fprintf('  Angular ω: %.3f rad/s (%.1f°/s)\n', omega, rad2deg(omega));
            fprintf('  Gyro bias: %.4f rad/s (%.2f°/s)\n', b, rad2deg(b));
            
            fprintf('\nUncertainty:\n');
            fprintf('  Total (trace P): %.4f\n', obj.get_uncertainty());
            fprintf('  Position std:    %.3f m\n', sqrt(trace(obj.get_position_covariance())));
            fprintf('  Velocity std:    %.3f m/s\n', sqrt(trace(obj.get_velocity_covariance())));
            
            fprintf('\nGPS Statistics:\n');
            fprintf('  Update count:    %d\n', obj.gps_update_count);
            fprintf('  Last update:     %.2f s\n', obj.gps_last_update);
            fprintf('  GPS noise (σ):   %.2f m (x), %.2f m (y)\n', ...
                    sqrt(obj.R_gps(1,1)), sqrt(obj.R_gps(2,2)));
            
            if ~isempty(obj.innovation)
                fprintf('\nLast Innovation:\n');
                fprintf('  Value: [');
                fprintf('%.4f ', obj.innovation);
                fprintf(']\n');
                if ~isempty(obj.S)
                    fprintf('  Mahalanobis: %.2f\n', sqrt(obj.innovation' / obj.S * obj.innovation));
                end
            end
            
            fprintf('========================================\n\n');
        end
    end
end