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
    % Motion model (discrete, Option B - CORRECTED to match NMPC):
    %
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
    %
    % CRITICAL FIXES APPLIED (v2.0):
    %   ✅ Actuator dynamics now use exponential filter (matches NMPC)
    %   ✅ Jacobian corrected for exponential filter discretization
    %   ✅ Process noise properly scaled by physical units
    %   ✅ Added get_state_for_controller() for NMPC interface
    %
    % ---------------------------------------------------------------------
    % Author: Nova Carter Team
    % Date: November 2025
    % Version: 2.0 (Option B Compatible)
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

        % diagnostics
        innovation      % last innovation vector
        S               % last innovation covariance
        K               % last Kalman gain
    end

    methods
        function obj = ekf_state_estimator_optionB(x0, P0, Q, R_enc, R_imu)
            % Constructor: initialize EKF with initial state/covariances
            %
            % INPUTS:
            %   x0    - Initial state [x; y; θ; v; ω; b] (6x1)
            %   P0    - Initial covariance (6x6)
            %   Q     - Process noise (6x6 or scalar)
            %   R_enc - Encoder noise (2x2 or scalar)
            %   R_imu - IMU noise (1x1 or scalar)
            %
            % EXAMPLE:
            %   x0 = [0; 0; 0; 0; 0; 0];
            %   P0 = diag([0.1, 0.1, 0.01, 0.1, 0.01, 0.001]);
            %   Q = 0.01;  % Will be scaled appropriately
            %   R_enc = 0.01;
            %   R_imu = 0.001;
            %   ekf = ekf_state_estimator_optionB(x0, P0, Q, R_enc, R_imu);

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
                % Scale by physical significance of each state
                % Units: [m², m², rad², (m/s)², (rad/s)², (rad/s)²]
                Q_diag = [ Q,          % x position (m²)
                           Q,          % y position (m²)
                           Q * 0.01,   % heading (rad²) - angles are small
                           Q,          % linear velocity (m²/s²)
                           Q * 0.01,   % angular velocity (rad²/s²) - angles
                           Q * 0.001]; % bias (rad²/s²) - very slow drift
                obj.Q = diag(Q_diag);
                
                fprintf('  EKF: Process noise scaled by units\n');
                fprintf('    Position:  %.4f m²/step\n', Q);
                fprintf('    Heading:   %.4f rad²/step (%.2f°²/step)\n', ...
                        Q*0.01, rad2deg(sqrt(Q*0.01))^2);
                fprintf('    Velocity:  %.4f (m/s)²/step\n', Q);
                fprintf('    Ang. vel:  %.4f (rad/s)²/step\n', Q*0.01);
                fprintf('    Bias:      %.6f (rad/s)²/step\n', Q*0.001);
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

            % 7) diagnostics init
            obj.innovation = [];
            obj.S          = [];
            obj.K          = [];
            
            fprintf('  EKF initialized: tau_v=%.3fs, tau_w=%.3fs\n', ...
                    obj.tau_v, obj.tau_w);
        end

        % -----------------------------------------------------------------
        function obj = predict(obj, u_cmd)
            % PREDICT  EKF prediction step with 1st-order actuator dynamics
            %
            % CORRECTED VERSION: Now uses exponential filter to match NMPC!
            %
            % INPUTS:
            %   u_cmd = [v_cmd; omega_cmd]  (2x1)
            %
            % DYNAMICS:
            %   Position:  x += v*cos(θ)*dt,  y += v*sin(θ)*dt
            %   Heading:   θ += ω*dt
            %   Actuator:  v_new = α_v*v_cmd + (1-α_v)*v
            %              ω_new = α_w*ω_cmd + (1-α_w)*ω
            %              where α = dt/(τ + dt)
            %   Bias:      b_new = b (constant)

            dt    = obj.dt;
            tau_v = obj.tau_v;
            tau_w = obj.tau_w;

            % --- Validate input ---
            if nargin < 2 || isempty(u_cmd)
                error('predict() requires control input u_cmd = [v_cmd; omega_cmd]');
            end
            
            if length(u_cmd) ~= 2
                error('u_cmd must be 2D [v_cmd; omega_cmd], got %dD', length(u_cmd));
            end

            % --- unpack state ---
            x  = obj.x_hat;
            px = x(1);
            py = x(2);
            th = x(3);
            v  = x(4);
            w  = x(5);
            b  = x(6);

            % --- unpack commands ---
            v_cmd = u_cmd(1);
            w_cmd = u_cmd(2);

            % ========== CORRECTED: Exponential filter coefficients ==========
            % This matches the NMPC controller exactly!
            % Transfer function: H(s) = 1/(τs + 1)
            % Discrete equivalent: α = dt/(τ + dt)
            alpha_v = dt / (tau_v + dt);
            alpha_w = dt / (tau_w + dt);

            % ===================== STATE PREDICTION ======================
            % Kinematics (using CURRENT velocities v, w)
            px_next = px + v * cos(th) * dt;
            py_next = py + v * sin(th) * dt;
            th_next = th + w * dt;
            
            % CORRECTED actuator dynamics (exponential filter)
            v_next = alpha_v * v_cmd + (1 - alpha_v) * v;
            w_next = alpha_w * w_cmd + (1 - alpha_w) * w;
            
            % Bias (constant model)
            b_next = b;

            % Update state estimate
            obj.x_hat = [px_next; py_next; th_next; v_next; w_next; b_next];

            % ========== CORRECTED JACOBIAN (F = df/dx) ==================
            % Partial derivatives of f(x, u) with respect to x
            %
            % State dynamics:
            %   f1 = x + v*cos(θ)*dt
            %   f2 = y + v*sin(θ)*dt
            %   f3 = θ + ω*dt
            %   f4 = α_v*v_cmd + (1-α_v)*v    ← CORRECTED!
            %   f5 = α_w*ω_cmd + (1-α_w)*ω    ← CORRECTED!
            %   f6 = b
            %
            % Jacobian elements:
            %   ∂f1/∂θ = -v*sin(θ)*dt
            %   ∂f1/∂v =  cos(θ)*dt
            %   ∂f2/∂θ =  v*cos(θ)*dt
            %   ∂f2/∂v =  sin(θ)*dt
            %   ∂f3/∂ω =  dt
            %   ∂f4/∂v = (1 - α_v)    ← CORRECTED! (was 1 - dt/tau_v)
            %   ∂f5/∂ω = (1 - α_w)    ← CORRECTED! (was 1 - dt/tau_w)

            F = [ 1, 0, -v*sin(th)*dt,  cos(th)*dt,         0,                0;
                  0, 1,  v*cos(th)*dt,  sin(th)*dt,         0,                0;
                  0, 0,  1,             0,                  dt,               0;
                  0, 0,  0,             1 - alpha_v,        0,                0;
                  0, 0,  0,             0,           1 - alpha_w,             0;
                  0, 0,  0,             0,                  0,                1];

            % Covariance prediction
            obj.P = F * obj.P * F' + obj.Q;
        end

        % -----------------------------------------------------------------
        function obj = update_encoders(obj, z_enc)
            % UPDATE_ENCODERS  EKF correction with wheel-speed-derived v, omega
            %
            % INPUTS:
            %   z_enc = [v_meas; omega_meas]  (2x1)
            %
            % Measurement model: z = H*x + noise
            %   where H extracts velocities from state

            % Measurement matrix
            H = [0, 0, 0, 1, 0, 0;   % measures v (state 4)
                 0, 0, 0, 0, 1, 0];  % measures ω (state 5)

            % Innovation
            z_pred = H * obj.x_hat;
            y      = z_enc - z_pred;
            
            % Innovation covariance
            S = H * obj.P * H' + obj.R_enc;
            
            % Kalman gain
            K = obj.P * H' / S;

            % State update
            obj.x_hat = obj.x_hat + K * y;

            % Covariance update (Joseph form for numerical stability)
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_enc * K';

            % Store diagnostics
            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
        end

        % -----------------------------------------------------------------
        function obj = update_imu(obj, z_imu)
            % UPDATE_IMU  EKF correction with gyro measurement
            %
            % INPUTS:
            %   z_imu - scalar gyro measurement (angular velocity)
            %
            % IMU model: z_imu = omega + b + noise
            %   where b is the gyro bias

            % Measurement matrix (extracts ω + b)
            H = [0, 0, 0, 0, 1, 1];  % measures state(5) + state(6)

            % Innovation
            z_pred = H * obj.x_hat;
            y      = z_imu - z_pred;
            
            % Innovation covariance
            S = H * obj.P * H' + obj.R_imu;
            
            % // Kalman gain
            K = obj.P * H' / S;

            % State update
            obj.x_hat = obj.x_hat + K * y;

            % Covariance update (Joseph form)
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_imu * K';

            % Store diagnostics
            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
        end

        % -----------------------------------------------------------------
        function obj = update_encoders_and_imu(obj, z_enc, z_imu)
            % UPDATE_ENCODERS_AND_IMU  EKF correction with BOTH sensors
            %
            % INPUTS:
            %   z_enc = [v_meas; omega_meas]  (2x1)
            %   z_imu - scalar gyro measurement
            %
            % Combined measurement:
            %   z = [v; ω_enc; ω_imu] where ω_imu = ω + b

            % Stack measurements
            z = [z_enc; z_imu];

            % Combined measurement matrix
            H = [0, 0, 0, 1, 0, 0;   % v from encoders
                 0, 0, 0, 0, 1, 0;   % ω from encoders
                 0, 0, 0, 0, 1, 1];  % ω + b from IMU

            % Combined noise covariance
            R = blkdiag(obj.R_enc, obj.R_imu);

            % Innovation
            z_pred = H * obj.x_hat;
            y      = z - z_pred;
            
            % Innovation covariance
            S = H * obj.P * H' + R;
            
            % Kalman gain
            K = obj.P * H' / S;

            % State update
            obj.x_hat = obj.x_hat + K * y;

            % Covariance update (Joseph form)
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * R * K';

            % Store diagnostics
            obj.innovation = y;
            obj.S          = S;
            obj.K          = K;
        end

        % -----------------------------------------------------------------
        % ✅ NEW METHOD: Interface for NMPC controller
        % -----------------------------------------------------------------
        function x_for_nmpc = get_state_for_controller(obj)
            % GET_STATE_FOR_CONTROLLER  Return 5D state for NMPC
            %
            % OUTPUT:
            %   x_for_nmpc = [x; y; θ; v; ω]  (5x1)
            %
            % This method extracts the first 5 states (position + velocities)
            % and drops the gyro bias, which is internal to the EKF.
            % The NMPC controller uses 5D state, so this provides compatibility.
            %
            % USAGE:
            %   ekf = ekf_state_estimator_optionB(...);
            %   ekf = ekf.predict(u_cmd);
            %   ekf = ekf.update_encoders(z_enc);
            %   
            %   x_current = ekf.get_state_for_controller();
            %   [u_opt, ~] = nmpc.solve(x_current, x_ref, u_last);
            
            x_for_nmpc = obj.x_hat(1:5);
        end

        % -----------------------------------------------------------------
        function b_est = get_gyro_bias(obj)
            % GET_GYRO_BIAS  Return estimated gyro bias
            %
            % OUTPUT:
            %   b_est - estimated bias (rad/s)
            %
            % Useful for diagnostics and sensor calibration
            
            b_est = obj.x_hat(6);
        end

        % -----------------------------------------------------------------
        function [x, y, theta, v, omega, b] = get_state(obj)
            % GET_STATE  Return full (unwrapped) state
            %
            % OUTPUTS:
            %   x     - x position (m)
            %   y     - y position (m)
            %   theta - heading, unwrapped (rad)
            %   v     - linear velocity (m/s)
            %   omega - angular velocity (rad/s)
            %   b     - gyro bias (rad/s)
            
            x     = obj.x_hat(1);
            y     = obj.x_hat(2);
            theta = obj.x_hat(3);
            v     = obj.x_hat(4);
            omega = obj.x_hat(5);
            b     = obj.x_hat(6);
        end

        % -----------------------------------------------------------------
        function [x, y, theta_wrapped, v, omega, b] = get_state_wrapped(obj)
            % GET_STATE_WRAPPED  Return state with heading wrapped to [-π, π]
            %
            % OUTPUTS:
            %   x              - x position (m)
            %   y              - y position (m)
            %   theta_wrapped  - heading, wrapped to [-π, π] (rad)
            %   v              - linear velocity (m/s)
            %   omega          - angular velocity (rad/s)
            %   b              - gyro bias (rad/s)
            %
            % Use this for visualization where you want angles in [-π, π]
            
            x              = obj.x_hat(1);
            y              = obj.x_hat(2);
            theta_wrapped  = wrapToPi(obj.x_hat(3));
            v              = obj.x_hat(4);
            omega          = obj.x_hat(5);
            b              = obj.x_hat(6);
        end

        % -----------------------------------------------------------------
        function cov_trace = get_uncertainty(obj)
            % GET_UNCERTAINTY  Return trace of covariance (scalar measure)
            %
            % OUTPUT:
            %   cov_trace - sum of diagonal elements of P
            %
            % Larger value = more uncertainty in state estimate
            
            cov_trace = trace(obj.P);
        end

        % -----------------------------------------------------------------
        function pos_cov = get_position_covariance(obj)
            % GET_POSITION_COVARIANCE  Return 2x2 covariance for (x,y)
            %
            % OUTPUT:
            %   pos_cov - 2x2 covariance matrix for position
            %
            % Useful for plotting uncertainty ellipses
            
            pos_cov = obj.P(1:2, 1:2);
        end
        
        % -----------------------------------------------------------------
        function vel_cov = get_velocity_covariance(obj)
            % GET_VELOCITY_COVARIANCE  Return 2x2 covariance for (v,ω)
            %
            % OUTPUT:
            %   vel_cov - 2x2 covariance matrix for velocities
            %
            % NEW: Useful for monitoring actuator state uncertainty
            
            vel_cov = obj.P(4:5, 4:5);
        end
        
        % -----------------------------------------------------------------
        function print_diagnostics(obj)
            % PRINT_DIAGNOSTICS  Display current filter state and statistics
            %
            % Shows:
            %   - Current state estimate
            %   - Uncertainty (trace of P)
            %   - Last innovation (if available)
            %   - Gyro bias estimate
            
            fprintf('\n========================================\n');
            fprintf('EKF State Estimator (Option B)\n');
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