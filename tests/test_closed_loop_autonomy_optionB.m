%% test_closed_loop_autonomy_optionB.m
%
% PURPOSE:
%   Complete closed-loop test for Option B (actuator dynamics)
%   ALIGNED WITH: test_closed_loop_autonomy.m (Option A)
%
% AUTHOR: Nova Carter NMPC Team
% DATE: November 2025

function test_closed_loop_autonomy_optionB()
    clc; close all;
    fprintf('\n======================================================\n');
    fprintf('PHASE 4B: CLOSED-LOOP AUTONOMY — EKF + NMPC + PLANT (Option B)\n');
    fprintf('======================================================\n\n');

    %% 1. Setup
    params = nova_carter_params();
    dt = params.dt;
    
    T_sim = 90.0;  % Match Option A duration
    N_steps = round(T_sim / dt);
    
    % Plant model (Option B - 5D with actuator dynamics)
    plant_model = differential_drive_model();
    plant_model.tau_v = 0.2;        % 200ms lag
    plant_model.tau_omega = 0.15;   % 150ms lag

    %% 2. NM1PC Parameters
    N_mpc = 15;  % Match Option A
    start_time = 2.0;

    x0   = 0;
    y0   = 0;
    th0  = 0;        % face +x
    v_f  = 0.4;      % m/s
    L1   = 40.0;     % 10 m straight
    R    = 10;      % 1.5 m turn radius (nice and smooth)
    L2   = 50.0;     % 15 m after the turn
    % 
    % x_ref = generate_L_reference(N_steps, N_mpc, dt, ...
    %     x0, y0, th0, ...
    %     v_f, L1, R, L2);
    radius = 8;

    x_ref = generate_spiral_reference(N_steps, N_mpc, dt, 0.5, 0.5);
    x_ref = generate_circular_reference(N_steps, N_mpc, dt, radius)

% CRITICAL: Lift to 5D for Option B NMPC
    x_ref = lift_reference_to_5d(x_ref, dt);


    % Generate reference (same as Option A, then lift to 5D)
    
    % Lift to 5D for Option B
    % x_ref = lift_reference_to_5d(x_ref_3d, dt);

    % Cost matrices (5D for Option B)
    Q_mpc = diag([40, 45, 30, 0.1, 0.1]);  % [x, y, θ, v, ω]
    R_mpc = diag([0.1, 0.5]);
    S_mpc = diag([1.0, 2.0]);
    Qf_mpc = 50 * Q_mpc;

    % Constraints (same as Option A)
    v_min = 0.0;
    v_max = 3.33;
    omega_min = -pi/2;
    omega_max = pi/2;
    u_min = [v_min; omega_min];
    u_max = [v_max; omega_max];

    a_max = 2.50;
    alpha_max = 3.0;
    du_max = [a_max * dt; alpha_max * dt];

    %% 3. EKF Parameters (6D for Option B)
P0_ekf = diag([0.5, 0.5, 0.2, 0.05, 0.02, 0.05].^2);

% % Increase Q_bias to tell the filter the bias is "drifty"
% Q_ekf = diag([
%     1e-5;   % x
%     1e-5;   % y
%     1e-1;   % θ
%     1e-4;   % v
%     1e-4;   % ω
%     1e-4    % bias (Increased from 1e-5)
% ]);
% 
% % AGGRESSIVELY INFLATE R (The Key Fix)
% % Tell the filter the sensors are 5-10x noisier than they are.
% R_enc = diag([0.1^2, 0.05^2]);  % Was [0.05, 0.02]
% R_imu = 0.05^2;                % Was 0.02



Q_ekf = diag([1e-4, 1e-4, 1e-3, 1e-2, 1e-2, 1e-3]);  % Process noise
R_enc = diag([0.9, 0.9]);                     % Encoder noise
R_imu = 0.9;                                   % IMU noise
% --- Get these values from your sensor_noise_simulator.m ---
enc_sigma_v = 0.01;
enc_sigma_omega = 0.005;
imu_sigma_gyro = 0.005;

% --- Set R to the SQUARE of the standard deviations ---
R_enc = diag([enc_sigma_v^2, enc_sigma_omega^2]);  % e.g., [1e-4, 2.5e-5]
R_imu = imu_sigma_gyro^2;

    %% 4. Initialization
    fprintf('  Initializing NMPC controller... ');
    tic;
    nmpc = nmpc_casadi_controller_optionB(N_mpc, Q_mpc, R_mpc, S_mpc, Qf_mpc, dt, ...
        u_min, u_max, du_max, plant_model.tau_v, plant_model.tau_omega);
    fprintf('MPC Initialisation Done (%.2fs)\n', toc);

    % EKF initialization (6D: [x; y; θ; v; ω; bias])
    x0_ekf = [x_ref(1:3, 1); 0; 0; 0];
    ekf = ekf_state_estimator_optionB(x0_ekf, P0_ekf, Q_ekf, R_enc, R_imu);

    noise_sim = sensor_noise_simulator('low', 'low');

    % True state (5D for Option B: [x; y; θ; v; ω])
    x_true = zeros(5, N_steps+1);
    x_true(:,1) = [x_ref(1:3, 1); 0; 0];  % Start aligned, at rest

    u_last = [0; 0];

    %% 5. Storage
    x_hat_history = zeros(5, N_steps+1);   % EKF 5D state for controller
    x_true_history = zeros(5, N_steps+1);
    x_est_history = zeros(6, N_steps+1);   % Full EKF state with bias
    u_history = zeros(2, N_steps);
    solve_times = zeros(1, N_steps);

    x_hat_history(:,1) = ekf.get_state_for_controller();
    x_est_history(:,1) = ekf.x_hat;
    x_true_history(:,1) = x_true(:,1);

    %% 6. Closed-Loop Simulation
    fprintf('  Running closed-loop simulation (%d steps)...\n', N_steps);
    fprintf('  Running closed-loop simulation (%d steps)...\n', N_steps);

% --- plant models ---
fk_model     = forward_kinematics();       % wheel → pose
dd_model     = differential_drive_model(); % for IK (wheel speeds), tau_v, tau_omega

% --- true states ---
x_fk_true    = x_ref(1:3,1);     % pose from FK (3x1): [x;y;θ]
v_true       = 0;                % actual forward velocity (lagged)
omega_true   = 0;                % actual yaw rate (lagged)

for k = 1:N_steps

    % ============================================================
    % 1) SENSOR MEASUREMENTS (from *actual* v, ω)
    % ============================================================
    z_enc = noise_sim.add_encoder_noise(v_true, omega_true)*0000001;
    z_imu = noise_sim.add_imu_noise(omega_true, k*dt)*0000001;

    % ============================================================
    % 2) EKF: predict with last command, then update with sensors
    %    (Option B EKF expects the *command* we sent to motors)
    % ============================================================
    ekf = ekf.predict(u_last);                     % u_last = [v_cmd; ω_cmd]
    ekf = ekf.update_encoders_and_imu(z_enc, z_imu);

    x_hat = ekf.get_state_for_controller();        % 5D: [x;y;θ; v; ω]
    x_est_history(:,k+1) = ekf.x_hat;

    % ============================================================
    % 3) NMPC: compute next command from estimate
    % ============================================================
    % k_end = min(k + N_mpc, size(x_ref, 2));
    % x_ref_segment = x_ref(:, k:k_end);
    % if size(x_ref_segment, 2) < N_mpc + 1
    %     n_pad = N_mpc + 1 - size(x_ref_segment, 2);
    %     x_ref_segment = [x_ref_segment, repmat(x_ref_segment(:,end), 1, n_pad)];
    % end
    % 
    % tic;
    % [u_cmd, ~] = nmpc.solve(x_hat, x_ref_segment, u_last);  % u_cmd = [v_cmd; ω_cmd]
    % solve_times(k) = toc;

    % --- NEW LINE: Create the "perfect state" for the NMPC ---
    % This is the state of the robot at the *start* of the step
    x_hat_perfect = [x_fk_true; v_true; omega_true];
    
    k_end = min(k + N_mpc, size(x_ref, 2));
    x_ref_segment = x_ref(:, k:k_end);
    if size(x_ref_segment, 2) < N_mpc + 1
        n_pad = N_mpc + 1 - size(x_ref_segment, 2);
        x_ref_segment = [x_ref_segment, repmat(x_ref_segment(:,end), 1, n_pad)];
    end
    
    tic;
    % --- MODIFIED LINE: Feed NMPC the *perfect* state ---
    [u_cmd, ~] = nmpc.solve(x_hat_perfect, x_ref_segment, u_last);  
    solve_times(k) = toc;

    % ============================================================
    % 4) PLANT SIDE (this is the pipeline you wanted)
    %
    %    NMPC (v_cmd, ω_cmd)
    %            ↓
    %       actuator lag (Option B)
    %            ↓
    %    inverse kinematics → wheel speeds
    %            ↓
    %    forward kinematics → x,y,θ  (this is x_true pose)
    % ============================================================

    % 4.a actuator lag (to get *actual* v, ω the robot can realize)
    alpha_v     = dt / (dd_model.tau_v     + dt);
    alpha_omega = dt / (dd_model.tau_omega + dt);

    v_true_next     = alpha_v     * u_cmd(1) + (1 - alpha_v)     * v_true;
    omega_true_next = alpha_omega * u_cmd(2) + (1 - alpha_omega) * omega_true;

    % 4.b inverse kinematics: chassis → wheels
    [phi_dot_L, phi_dot_R] = dd_model.convert_to_wheel_speeds([v_true_next; omega_true_next]);

    % 4.c forward kinematics: wheels → pose
    x_fk_next = fk_model.propagate_from_wheels( ...
        x_fk_true, phi_dot_L, phi_dot_R, dt);

    % ============================================================
    % 5) STORE + ADVANCE
    % ============================================================
    % pack into 5D so the rest of your code (plots, errors) still works
    % x_true(:,k+1)        = [x_fk_next; v_true_next; omega_true_next];
    % x_true_history(:,k+1)= x_true(:,k+1);
    % x_hat_history(:,k+1) = x_hat;
    % u_history(:,k)       = u_cmd;
    % u_last               = u_cmd;
    % 
    % % advance the "physical" states
    % x_fk_true  = x_fk_next;
    % v_true     = v_true_next;
    % omega_true = omega_true_next;
    % x_true_5d(:,k+1) = [x_fk_next; v_true_next; omega_true_next];
    % pack into 5D
    x_true(:,k+1)        = [x_fk_next; v_true_next; omega_true_next];
    x_true_history(:,k+1)= x_true(:,k+1);
    
    % --- MODIFIED LINE: Store the "perfect" state you used ---
    x_hat_history(:,k+1) = x_hat_perfect; 
    
    u_history(:,k)       = u_cmd;
    u_last               = u_cmd;
    
    % advance the "physical" states
    x_fk_true  = x_fk_next;
    v_true     = v_true_next;
    omega_true = omega_true_next;
end

fprintf('  ✓ Simulation complete\n');

    %% 7. Analysis (Match Option A format exactly)
    % Extract 3D states for plotting compatibility
    x_true_3d = x_true_history(1:3, :);
    x_hat_3d = x_hat_history(1:3, :);
    x_ref_3d = x_ref(1:3, :);
    
    plot_tracking_results(x_true_3d, u_history, x_ref_3d, dt, ...
        'Closed-Loop Autonomy (Option B)', x_est_history(3,:));
    plot_ekf_estimation_error(x_true, x_est_history(1:5,:), dt);

    
    % Calculate errors (same as Option A)
    N_steps_actual = size(x_true_3d, 2) - 1;
    x_ref_synced = x_ref_3d(:, 1:N_steps_actual+1);
    
    pos_tracking_error = compute_tracking_error(x_true_3d, x_ref_3d, dt, start_time);
    heading_error_rad = compute_heading_error(x_true_3d, x_ref_synced, dt, start_time);
    heading_error_deg = rad2deg(heading_error_rad);
    
    % Analyze and Print (Match Option A format)
    avg_solve_time_ms = mean(solve_times) * 1000;
    max_solve_time_ms = max(solve_times) * 1000;
    
    fprintf('  Final Position Error (RMSE): %.2f cm\n', pos_tracking_error * 100);
    fprintf('  Final Heading Error (RMSE): %.2f deg\n', heading_error_deg);
    fprintf('  Avg solve time: %.2f ms\n', avg_solve_time_ms);
    fprintf('  Max solve time: %.2f ms\n', max_solve_time_ms);
    
    if max_solve_time_ms < dt * 1000
        fprintf('  ✓ PASS: Real-time capable (Max < %.0f ms)\n', dt * 1000);
    else
        fprintf('  ✗ FAIL: Solver too slow (Max > %.0f ms)\n', dt * 1000);
    end
    
    fprintf('\n======================================================\n');
    fprintf('PHASE 4B COMPLETE: CLOSED-LOOP AUTONOMY VALIDATED (Option B)\n');
    fprintf('======================================================\n\n');
end

%% ==========================================================================
%  HELPER FUNCTIONS (Reuse from Option A)
%  ==========================================================================

function x_ref = generate_L_reference( ...
        N_steps, N_horizon, dt, ...
        x0, y0, th0, ...
        v_forward, L1, turn_radius, L2)

% SEGMENTS (in robot/world frame):
% 1) go straight L1
% 2) 90° left turn with given radius
% 3) go straight L2

    N_total = N_steps + N_horizon + 1;
    x_ref   = zeros(3, N_total);

    % --- segment durations ---
    t1 = L1 / v_forward;                   % straight 1
    turn_angle = pi/2;                     % 90 deg
    w_turn = v_forward / turn_radius;      % kinematic: v = R * w
    t2 = turn_angle / w_turn;              % time to do 90 deg
    t3 = L2 / v_forward;                   % straight 2
    T_total = t1 + t2 + t3;

    for k = 1:N_total
        t = (k-1) * dt;

        % clamp final time
        if t > T_total
            t = T_total;
        end

        if t <= t1
            % --- SEGMENT 1: straight ---
            s = t;  % time on this segment
            x = x0 + v_forward * s * cos(th0);
            y = y0 + v_forward * s * sin(th0);
            heading = th0;

        elseif t <= t1 + t2
            % --- SEGMENT 2: 90° arc to the left ---
            s  = t - t1;            % time in turn
            ang = w_turn * s;       % 0 -> 90deg

            % center of turn is on the LEFT of current heading
            % initial heading is th0
            cx = x0 + L1 * cos(th0) - turn_radius * sin(th0);
            cy = y0 + L1 * sin(th0) + turn_radius * cos(th0);

            % position on arc
            x = cx + turn_radius * sin(th0 + ang);
            y = cy - turn_radius * cos(th0 + ang);

            heading = wrapToPi(th0 + ang);

        else
            % --- SEGMENT 3: straight after turn ---
            s  = t - (t1 + t2);     % time in segment 3
            x_turn_end = x0 + L1 * cos(th0) ...
                           + turn_radius * (sin(th0 + pi/2) - sin(th0));
            y_turn_end = y0 + L1 * sin(th0) ...
                           - turn_radius * (cos(th0 + pi/2) - cos(th0));

            % after the 90° left turn, heading is th0 + 90°
            th_straight = wrapToPi(th0 + pi/2);

            x = x_turn_end + v_forward * s * cos(th_straight);
            y = y_turn_end + v_forward * s * sin(th_straight);
            heading = th_straight;
        end

        x_ref(:,k) = [x; y; heading];
    end
end


function plot_tracking_results(x_history, u_history, x_ref, dt, title_str, ekf_heading)
    % Plot trajectory tracking results (EXACT COPY from Option A)
    
    N = size(x_history, 2);
    t = (0:N-1) * dt;
    t_u = (0:size(u_history,2)-1) * dt;
    
    figure('Name', title_str, 'Position', [100 100 1200 800]);
    
    % 2D Trajectory
    subplot(2,3,1);
    plot(x_ref(1,1:N), x_ref(2,1:N), 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    plot(x_history(1,1), x_history(2,1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Start');
    plot(x_history(1,end), x_history(2,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'End');
    grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)');
    title('2D Trajectory');
    legend('Location', 'best');
    
    % Position vs Time
    subplot(2,3,2);
    plot(t, x_ref(1,1:N), 'r--', 'LineWidth', 1.5); hold on;
    plot(t, x_history(1,:), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('X (m)');
    title('X Position');
    legend('Reference', 'Actual');
    
    subplot(2,3,3);
    plot(t, x_ref(2,1:N), 'r--', 'LineWidth', 1.5); hold on;
    plot(t, x_history(2,:), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('Y (m)');
    title('Y Position');
    
    % Heading
    subplot(2,3,4);
    plot(t, rad2deg(x_ref(3,1:N)), 'r--', 'LineWidth', 1.0, 'DisplayName', 'Reference (wrapped)');
    hold on;
    plot(t, rad2deg(wrapToPi(x_history(3,1:N))), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual (wrapped)');
    plot(t, rad2deg(wrapToPi(ekf_heading(1:N))), 'g-', 'LineWidth', 1.0, 'DisplayName', 'EKF (wrapped)');
    grid on;
    xlabel('Time (s)');
    ylabel('Heading (deg)');
    title('Heading (wrapped)');
    legend('Location','best');
        
    % Control: Linear Velocity
    subplot(2,3,5);
    plot(t_u, u_history(1,:), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('v (m/s)');
    title('Linear Velocity Command');
    ylim([0, 1.5]);
    
    % Control: Angular Velocity
    subplot(2,3,6);
    plot(t_u, rad2deg(u_history(2,:)), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('ω (deg/s)');
    title('Angular Velocity Command');

    % Error plots (separate figure)
    Nerr = min(size(x_history,2), size(x_ref,2));
    t_err = (0:Nerr-1) * dt;

    ex  = x_history(1,1:Nerr) - x_ref(1,1:Nerr);
    ey  = x_history(2,1:Nerr) - x_ref(2,1:Nerr);
    eth = wrapToPi(x_history(3,1:Nerr) - x_ref(3,1:Nerr));

    figure('Name','Tracking errors');
    
    subplot(3,1,1);
    plot(t_err, ex, 'LineWidth', 1.8);
    grid on;
    ylabel('e_x (m)');
    title('Position and heading tracking errors');

    subplot(3,1,2);
    plot(t_err, ey, 'LineWidth', 1.8);
    grid on;
    ylabel('e_y (m)');

    subplot(3,1,3);
    plot(t_err, rad2deg(eth), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_\theta (deg)');
end


function err = compute_tracking_error(x_actual, x_ref, dt, start_time)
    % EXACT COPY from Option A
    start_idx = ceil(start_time / dt);
    N = size(x_actual, 2);
    errors_sq = zeros(1, N - start_idx + 1);

    for k = start_idx:N
        actual_pos = x_actual(1:2, k);
        ref_pos_all = x_ref(1:2, :);
        dists = vecnorm(ref_pos_all - actual_pos, 2, 1);
        min_dist = min(dists);
        errors_sq(k - start_idx + 1) = min_dist^2;
    end

    err = sqrt(mean(errors_sq));
end

function heading_err = compute_heading_error(x_actual, x_ref, dt, start_time)
    % EXACT COPY from Option A
    start_idx = ceil(start_time / dt);
    N_actual = size(x_actual, 2);
    N_ref = size(x_ref, 2);
    N_common = min(N_actual, N_ref);
    idx_range = start_idx:N_common;

    e_theta = x_actual(3,idx_range) - x_ref(3,idx_range);
    e_theta_wrapped = wrapToPi(e_theta);
    heading_err = sqrt(mean(e_theta_wrapped.^2));
end

function wrapped = wrapToPi(angle)
    % EXACT COPY from Option A
    wrapped = angle - 2*pi * floor((angle + pi) / (2*pi));
end

function x_ref = generate_spiral_reference(N_steps, N_horizon, dt, growth_rate, angular_velocity)
    % Generate an outward spiral reference trajectory
    % Robot spirals counter-clockwise with increasing radius

    N_total = N_steps + N_horizon + 1;
    x_ref = zeros(3, N_total);

    for k = 1:N_total
        t = (k-1) * dt;
        r = growth_rate * t;              % Radius increases linearly
        theta = angular_velocity * t;     % Constant angular speed
        x = r * cos(theta);
        y = r * sin(theta);
        heading = theta + pi/2;
        heading = wrapToPi(theta + pi/2);
        % Tangent to spiral
        x_ref(:,k) = [x; y; heading];
    end
end


function x_ref = generate_circular_reference(N_steps, N_horizon, dt, radius)
    % Generate a circular reference trajectory
    % Robot moves counter-clockwise around a circle of given radius

    N_total = N_steps + N_horizon + 1;
    omega = 1.0 / radius;  % Constant angular velocity
    v = omega * radius;    % Constant linear velocity

    x_ref = zeros(3, N_total);
    for k = 1:N_total
        t = (k-1) * dt;
        theta = pi/2 + omega * t;  % Start at top of circle
        x = radius * cos(theta);
        y = radius * sin(theta);
        heading = theta + pi/2;  % Tangent to the circle (no wrap)
        heading = wrapToPi(theta + pi/2);
        x_ref(:,k) = [x; y; heading];
    end
end


function x_ref_5d = lift_reference_to_5d(x_ref_3d, dt)
    % Lift 3D reference to 5D by computing velocities
    %
    % INPUTS:
    %   x_ref_3d - 3×N reference [x; y; θ]
    %   dt       - time step (s)
    %
    % OUTPUT:
    %   x_ref_5d - 5×N reference [x; y; θ; v; ω]
    
    N = size(x_ref_3d, 2);
    x_ref_5d = zeros(5, N);
    x_ref_5d(1:3, :) = x_ref_3d;
    
    % Compute reference velocities via finite differences
    for k = 1:N-1
        dx = x_ref_3d(1, k+1) - x_ref_3d(1, k);
        dy = x_ref_3d(2, k+1) - x_ref_3d(2, k);
        v_ref = sqrt(dx^2 + dy^2) / dt;
        x_ref_5d(4, k) = v_ref;
        
        dtheta = wrapToPi(x_ref_3d(3, k+1) - x_ref_3d(3, k));
        omega_ref = dtheta / dt;
        x_ref_5d(5, k) = omega_ref;
    end
    
    % Last sample: repeat previous
    x_ref_5d(4, end) = x_ref_5d(4, end-1);
    x_ref_5d(5, end) = x_ref_5d(5, end-1);
end



function plot_ekf_estimation_error(x_history, x_ekf_history, dt)
    % Plot EKF estimation error over time
    N = min(size(x_history,2), size(x_ekf_history,2));
    t = (0:N-1) * dt;

    ex = x_ekf_history(1,1:N) - x_history(1,1:N);
    ey = x_ekf_history(2,1:N) - x_history(2,1:N);
    eth = wrapToPi(x_ekf_history(3,1:N) - x_history(3,1:N));
    ev = x_ekf_history(4,1:N) - x_history(4,1:N);
    eomega = x_ekf_history(5,1:N) - x_history(5,1:N);

    figure('Name','EKF Estimation Error');
    
    subplot(3,2,1);
    plot(t, ex, 'LineWidth', 1.5);
    grid on; ylabel('e_x (m)'); title('Position Error');

    subplot(3,2,2);
    plot(t, ey, 'LineWidth', 1.5);
    grid on; ylabel('e_y (m)');

    subplot(3,2,3);
    plot(t, rad2deg(eth), 'LineWidth', 1.5);
    grid on; ylabel('e_\theta (deg)'); title('Heading Error');

    subplot(3,2,4);
    plot(t, ev, 'LineWidth', 1.5);
    grid on; ylabel('e_v (m/s)'); title('Velocity Error');

    subplot(3,2,5);
    plot(t, rad2deg(eomega), 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('e_\omega (deg/s)'); title('Angular Velocity Error');

    % rmse_x = sqrt(mean((x_est_history(1,:) - x_true_5d(1,:)).^2));
    % rmse_y = sqrt(mean((x_est_history(2,:) - x_true_5d(2,:)).^2));
    % rmse_theta = sqrt(mean(wrapToPi(x_est_history(3,:) - x_true_5d(3,:)).^2));
    % rmse_v = sqrt(mean((x_est_history(4,:) - x_true_5d(4,:)).^2));
    % rmse_omega = sqrt(mean((x_est_history(5,:) - x_true_5d(5,:)).^2));
    % 
    % fprintf('EKF RMSE:\n  e_x = %.3f m\n  e_y = %.3f m\n  e_θ = %.2f deg\n  e_v = %.3f m/s\n  e_ω = %.2f deg/s\n', ...
    %     rmse_x, rmse_y, rad2deg(rmse_theta), rmse_v, rad2deg(rmse_omega));
end

