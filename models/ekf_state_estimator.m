% %% ekf_state_estimator.m
% % Extended Kalman Filter for differential drive robot state estimation
% %
% % PURPOSE:
% %   Fuse noisy wheel encoder and IMU data to estimate robot state
% %   This replicates what robot_localization does in ROS2
% %
% % STATE VECTOR: x = [x, y, θ, v, ω]^T  (5D)
% %   x, y   - position (m)
% %   θ      - heading (rad)
% %   v      - linear velocity (m/s)
% %   ω      - angular velocity (rad/s)
% %
% % WHY 5D STATE (not 3D)?
% %   - NMPC needs velocity estimates, not just position
% %   - Wheel encoders measure velocities directly
% %   - Allows velocity-based control
% %
% % MEASUREMENTS:
% %   z_enc = [v_enc, ω_enc]^T  - from wheel encoders
% %   z_imu = ω_imu             - from IMU gyroscope
% %
% % PROCESS MODEL:
% %   Constant velocity model (velocities persist unless changed)
% %   x_{k+1} = x_k + v_k cos(θ_k) Δt
% %   y_{k+1} = y_k + v_k sin(θ_k) Δt
% %   θ_{k+1} = θ_k + ω_k Δt
% %   v_{k+1} = v_k + process_noise
% %   ω_{k+1} = ω_k + process_noise
% 
% classdef ekf_state_estimator
%     properties
%         params          % Robot parameters
% 
%         % State estimate
%         x_hat           % State estimate [x; y; theta; v; omega] (5x1)
%         P               % State covariance (5x5)
% 
%         % Process noise covariance
%         Q               % Process noise (5x5)
% 
%         % Measurement noise covariances
%         R_enc           % Encoder noise (2x2)
%         R_imu           % IMU gyro noise (scalar)
% 
%         dt              % Time step
% 
%         % For diagnostics
%         innovation      % Last innovation (measurement residual)
%         S               % Innovation covariance
%         K               % Last Kalman gain
%     end
% 
%     methods
%         function obj = ekf_state_estimator(x0, P0, Q, R_enc, R_imu)
%             % Constructor
%             %
%             % INPUTS:
%             %   x0    - initial state [x; y; theta; v; omega]
%             %   P0    - initial covariance (5x5) [how uncertain we are initially]
%             %   Q     - process noise covariance (5x5) or scalar
%             %           [how much we trust our motion model]
%             %   R_enc - encoder measurement noise (2x2) or scalar
%             %           [how noisy are wheel encoders]
%             %   R_imu - IMU gyro measurement noise (scalar)
%             %           [how noisy is IMU gyroscope]
%             %
%             % TUNING GUIDANCE:
%             %   Large Q → trust measurements more
%             %   Small Q → trust model more
%             %   Large R → trust measurements less
%             %   Small R → trust measurements more
% 
%             obj.params = nova_carter_params();
%             obj.dt = obj.params.dt;
% 
%             % Initialize state
%             obj.x_hat = x0;
%             obj.P = P0;
% 
%             % Process noise
%             if isscalar(Q)
%                 obj.Q = Q * eye(5);
%             else
%                 obj.Q = Q;
%             end
% 
%             % Measurement noise - encoders
%             if isscalar(R_enc)
%                 obj.R_enc = R_enc * eye(2);
%             else
%                 obj.R_enc = R_enc;
%             end
% 
%             % Measurement noise - IMU
%             obj.R_imu = R_imu;
% 
%             % Initialize diagnostics
%             obj.innovation = [];
%             obj.S = [];
%             obj.K = [];
%         end
% 
%         function obj = predict(obj, u)
%             % EKF PREDICTION STEP
%             % Propagate state and covariance forward in time using process model
%             %
%             % INPUT:
%             %   u - control input [v_cmd; omega_cmd] (optional, currently unused)
%             %
%             % MATH:
%             %   x̂_{k|k-1} = f(x̂_{k-1|k-1})     [state prediction]
%             %   P_{k|k-1} = F P_{k-1|k-1} F^T + Q  [covariance prediction]
% 
%             % Extract current state
%             x_pos = obj.x_hat(1);
%             y_pos = obj.x_hat(2);
%             theta = obj.x_hat(3);
%             v = obj.x_hat(4);
%             omega = obj.x_hat(5);
% 
%             % PROCESS MODEL: Constant velocity
%             % Robot continues at current velocity unless measurements correct it
%             x_next = x_pos + v * cos(theta) * obj.dt;
%             y_next = y_pos + v * sin(theta) * obj.dt;
%             theta_next = theta + omega * obj.dt;
%             v_next = v;      %// Assume velocity doesn't change
%             omega_next = omega;  %// Assume angular velocity doesn't change
% 
%             % Wrap angle to [-π, π]
%             theta_next = obj.params.wrapToPi(theta_next);
% 
%             % Predicted state
%             obj.x_hat = [x_next; y_next; theta_next; v_next; omega_next];
% 
%             % JACOBIAN: F = ∂f/∂x
%             % This linearizes the nonlinear process model around current state
%             %
%             % Why needed? EKF approximates nonlinear system as locally linear
%             %
%             %        ∂x/∂x  ∂x/∂y  ∂x/∂θ           ∂x/∂v        ∂x/∂ω
%             % F = [    1      0    -v·sin(θ)·Δt   cos(θ)·Δt      0     ]
%             %     [    0      1     v·cos(θ)·Δt   sin(θ)·Δt      0     ]
%             %     [    0      0         1              0         Δt     ]
%             %     [    0      0         0              1          0     ]
%             %     [    0      0         0              0          1     ]
% 
%             F = [1, 0, -v*sin(theta)*obj.dt, cos(theta)*obj.dt, 0;
%                  0, 1,  v*cos(theta)*obj.dt, sin(theta)*obj.dt, 0;
%                  0, 0,  1,                    0,                 obj.dt;
%                  0, 0,  0,                    1,                 0;
%                  0, 0,  0,                    0,                 1];
% 
%             % COVARIANCE PREDICTION
%             % How uncertainty grows due to process noise
%             obj.P = F * obj.P * F' + obj.Q;
%         end
% 
%         function obj = update_encoders(obj, z_enc)
%             % EKF UPDATE STEP - Encoder Measurements
%             % Correct state estimate using wheel encoder readings
%             %
%             % INPUT:
%             %   z_enc - [v_measured; omega_measured] from encoders
%             %
%             % MATH:
%             %   y = z - h(x̂)           [innovation/residual]
%             %   S = H P H^T + R         [innovation covariance]
%             %   K = P H^T S^{-1}        [Kalman gain]
%             %   x̂ = x̂ + K y            [state update]
%             %   P = (I - K H) P         [covariance update]
% 
%             % MEASUREMENT MODEL: Encoders observe v and ω directly
%             % h(x) = [v; ω] = [x(4); x(5)]
%             %
%             % H = ∂h/∂x (Jacobian)
%             H = [0, 0, 0, 1, 0;   % v is 4th state
%                  0, 0, 0, 0, 1];  % ω is 5th state
% 
%             % Predicted measurement (what we expect to measure)
%             z_pred = H * obj.x_hat;
% 
%             % INNOVATION: difference between actual and predicted measurement
%             % This tells us how wrong our prediction was
%             y = z_enc - z_pred;
% 
%             % INNOVATION COVARIANCE: how much we expect innovation to vary
%             S = H * obj.P * H' + obj.R_enc;
% 
%             % KALMAN GAIN: optimal weighting between prediction and measurement
%             % High K → trust measurement more
%             % Low K → trust prediction more
%             K = obj.P * H' / S;
% 
%             % STATE UPDATE: correct prediction with measurement
%             obj.x_hat = obj.x_hat + K * y;
% 
%             % Wrap angle
%             obj.x_hat(3) = obj.params.wrapToPi(obj.x_hat(3));
% 
%             % COVARIANCE UPDATE (Joseph form for numerical stability)
%             % Reduces uncertainty because we got new information
%             I = eye(5);
%             obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_enc * K';
% 
%             % Store diagnostics
%             obj.innovation = y;
%             obj.S = S;
%             obj.K = K;
%         end
% 
%         function obj = update_imu(obj, z_imu)
%             % EKF UPDATE STEP - IMU Gyroscope
%             % Correct angular velocity estimate using IMU
%             %
%             % INPUT:
%             %   z_imu - angular velocity from IMU (scalar)
%             %
%             % WHY SEPARATE FROM ENCODERS?
%             %   - IMU and encoders have different noise characteristics
%             %   - Can be called at different rates
%             %   - Allows sensor fusion flexibility
% 
%             % MEASUREMENT MODEL: IMU measures ω
%             H = [0, 0, 0, 0, 1];  % ω is 5th state
% 
%             % Predicted measurement
%             z_pred = H * obj.x_hat;
% 
%             % Innovation
%             y = z_imu - z_pred;
% 
%             % Innovation covariance
%             S = H * obj.P * H' + obj.R_imu;
% 
%             % Kalman gain
%             K = obj.P * H' / S;
% 
%             % State update
%             obj.x_hat = obj.x_hat + K * y;
% 
%             % Wrap angle
%             obj.x_hat(3) = obj.params.wrapToPi(obj.x_hat(3));
% 
%             % Covariance update
%             I = eye(5);
%             obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_imu * K';
% 
%             % Store diagnostics
%             obj.innovation = y;
%             obj.S = S;
%             obj.K = K;
%         end
% 
%         function obj = update_encoders_and_imu(obj, z_enc, z_imu)
%             % COMBINED UPDATE - Both sensors simultaneously
%             % More efficient than sequential updates
%             %
%             % INPUTS:
%             %   z_enc - [v_enc; omega_enc]
%             %   z_imu - omega_imu
%             %
%             % WHY COMBINED?
%             %   - Computationally more efficient
%             %   - Handles correlations better
%             %   - Single Kalman gain calculation
% 
%             % Stack measurements: [v_enc; ω_enc; ω_imu]
%             z = [z_enc; z_imu];
% 
%             % Combined measurement model
%             H = [0, 0, 0, 1, 0;   % v from encoders
%                  0, 0, 0, 0, 1;   % ω from encoders
%                  0, 0, 0, 0, 1];  % ω from IMU
% 
%             % Predicted measurement
%             z_pred = H * obj.x_hat;
% 
%             % Innovation
%             y = z - z_pred;
% 
%             % Combined measurement noise
%             R = blkdiag(obj.R_enc, obj.R_imu);
% 
%             % Innovation covariance
%             S = H * obj.P * H' + R;
% 
%             % Kalman gain
%             K = obj.P * H' / S;
% 
%             % State update
%             obj.x_hat = obj.x_hat + K * y;
% 
%             % Wrap angle
%             obj.x_hat(3) = obj.params.wrapToPi(obj.x_hat(3));
% 
%             % Covariance update (Joseph form)
%             I = eye(5);
%             obj.P = (I - K*H) * obj.P * (I - K*H)' + K * R * K';
% 
%             % Store diagnostics
%             obj.innovation = y;
%             obj.S = S;
%             obj.K = K;
%         end
% 
%         function [x, y, theta, v, omega] = get_state(obj)
%             % Extract state components
%             x = obj.x_hat(1);
%             y = obj.x_hat(2);
%             theta = obj.x_hat(3);
%             v = obj.x_hat(4);
%             omega = obj.x_hat(5);
%         end
% 
%         function cov_trace = get_uncertainty(obj)
%             % Get total uncertainty (trace of covariance matrix)
%             % Useful for monitoring filter convergence
%             cov_trace = trace(obj.P);
%         end
% 
%         function pos_cov = get_position_covariance(obj)
%             % Get 2x2 position covariance [x, y]
%             % Useful for plotting uncertainty ellipses
%             pos_cov = obj.P(1:2, 1:2);
%         end
%     end
% end
% 
% %


classdef ekf_state_estimator
    % Extended Kalman Filter for differential-drive robot
    % State: [x; y; theta; v; omega; b]
    % - x, y: position
    % - theta: heading (unwrapped)
    % - v: linear velocity
    % - omega: angular velocity
    % - b: gyroscope bias

    properties
        params          % Robot parameters (e.g., wheel radius, track width)
        
        x_hat           % Current state estimate (6x1)
        P               % State covariance matrix (6x6)
        
        Q               % Process noise covariance (6x6)
        R_enc           % Encoder measurement noise (2x2)
        R_imu           % IMU measurement noise (scalar)
        
        dt              % Time step (from params)
        
        % Diagnostics
        innovation      % Measurement residual
        S               % Innovation covariance
        K               % Kalman gain
    end
    
    methods
        function obj = ekf_state_estimator(x0, P0, Q, R_enc, R_imu)
            % Constructor: initializes EKF with initial state and covariances
            obj.params = nova_carter_params();
            obj.dt = obj.params.dt;
            
            obj.x_hat = x0;  % Initial state [x; y; theta; v; omega; b]
            obj.P = P0;      % Initial covariance
            
            % Handle scalar or full matrix input for Q and R
            % Handle scalar or full matrix input for Q
            if isscalar(Q)
                obj.Q = Q * eye(6);
            else
                obj.Q = Q;
            end
            
            % Handle scalar or full matrix input for R_enc
            if isscalar(R_enc)
                obj.R_enc = R_enc * eye(2);
            else
                obj.R_enc = R_enc;
            end
            
            % IMU noise is always scalar
            obj.R_imu = R_imu;
            
            % Initialize diagnostics
            obj.innovation = [];
            obj.S = [];
            obj.K = [];
        end
                
        function obj = predict(obj, u)
            % EKF PREDICTION STEP using differential drive kinematics
        
            % Extract current state
            x = obj.x_hat;
            x_pos = x(1);
            y_pos = x(2);
            theta = x(3);      % Unwrapped heading
            v = x(4);          % Linear velocity
            omega = x(5);      % Angular velocity
            b = x(6);          % Gyro bias
        
            % Predict next state using differential drive motion
            x_next = x_pos + v * cos(theta) * obj.dt;
            y_next = y_pos + v * sin(theta) * obj.dt;
            theta_next = theta + omega * obj.dt;
            v_next = v;
            omega_next = omega;
            b_next = b;  % Bias assumed constant
        
            obj.x_hat = [x_next; y_next; theta_next; v_next; omega_next; b_next];
        
            % Jacobian of the motion model (F)
            F = [1, 0, -v*sin(theta)*obj.dt, cos(theta)*obj.dt, 0, 0;
                 0, 1,  v*cos(theta)*obj.dt, sin(theta)*obj.dt, 0, 0;
                 0, 0,  1,                   0,                 obj.dt, 0;
                 0, 0,  0,                   1,                 0,      0;
                 0, 0,  0,                   0,                 1,      0;
                 0, 0,  0,                   0,                 0,      1];
        
            % Covariance prediction
            obj.P = F * obj.P * F' + obj.Q;
        end
        
        function obj = update_encoders(obj, z_enc)
            % EKF update using encoder measurements: [v; omega]
            H = [0, 0, 0, 1, 0, 0;
                 0, 0, 0, 0, 1, 0];
            
            z_pred = H * obj.x_hat;         % Predicted measurement
            y = z_enc - z_pred;             % Innovation
            S = H * obj.P * H' + obj.R_enc; % Innovation covariance
            K = obj.P * H' / S;             % Kalman gain
            
            obj.x_hat = obj.x_hat + K * y;  % State update
            
            % Joseph form covariance update
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_enc * K';
            
            % Store diagnostics
            obj.innovation = y;
            obj.S = S;
            obj.K = K;
        end
        
        function obj = update_imu(obj, z_imu)
            % EKF update using IMU gyroscope: measures omega + bias
            H = [0, 0, 0, 0, 1, 1];  % Measurement model: omega + bias
            
            z_pred = H * obj.x_hat;
            y = z_imu - z_pred;
            S = H * obj.P * H' + obj.R_imu;
            K = obj.P * H' / S;
            
            obj.x_hat = obj.x_hat + K * y;
            
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * obj.R_imu * K';
            
            obj.innovation = y;
            obj.S = S;
            obj.K = K;
        end
        
        function obj = update_encoders_and_imu(obj, z_enc, z_imu)
            % EKF update using both encoder and IMU measurements
            z = [z_enc; z_imu];  % Combined measurement
            
            % Combined measurement model
            H = [0, 0, 0, 1, 0, 0;
                 0, 0, 0, 0, 1, 0;
                 0, 0, 0, 0, 1, 1];
            R = blkdiag(obj.R_enc, obj.R_imu);  % Combined noise
            
            z_pred = H * obj.x_hat;
            y = z - z_pred;
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            obj.x_hat = obj.x_hat + K * y;
            
            I = eye(6);
            obj.P = (I - K*H) * obj.P * (I - K*H)' + K * R * K';
            
            obj.innovation = y;
            obj.S = S;
            obj.K = K;
        end
        
        function [x, y, theta, v, omega, b] = get_state(obj)
            % Return full state (unwrapped heading)
            x = obj.x_hat(1);
            y = obj.x_hat(2);
            theta = obj.x_hat(3);
            v = obj.x_hat(4);
            omega = obj.x_hat(5);
            b = obj.x_hat(6);
        end
        
        function [x, y, theta_wrapped, v, omega, b] = get_state_wrapped(obj)
            % Return state with wrapped heading (for display)
            x = obj.x_hat(1);
            y = obj.x_hat(2);
            theta_wrapped = wrapToPi(obj.x_hat(3));
            v = obj.x_hat(4);
            omega = obj.x_hat(5);
            b = obj.x_hat(6);
        end
        
        function cov_trace = get_uncertainty(obj)
            % Return total uncertainty (trace of covariance)
            cov_trace = trace(obj.P);
        end
        
        function pos_cov = get_position_covariance(obj)
            % Return 2x2 position covariance matrix
            pos_cov = obj.P(1:2, 1:2);
        end
    end
end