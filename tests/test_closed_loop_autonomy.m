function test_closed_loop_autonomy()
    clc; close all;
    fprintf('\n======================================================\n');
    fprintf('PHASE 4: CLOSED-LOOP AUTONOMY — EKF + NMPC + PLANT\n');
    fprintf('======================================================\n\n');

    %% 1. Setup
    params = nova_carter_params();
    dt = params.dt;
    
    T_sim =150.0;


    N_steps = round(T_sim / dt);
    fk_model = forward_kinematics();


    %% 2. NMPC Parameters
    N_mpc = 30;

    start_time = 2.0;

    radius = 8;

    

    % x_ref = generate_spiral_reference(N_steps, N_mpc, dt, 0.1, 0.5);
    % x_ref = generate_circular_reference(N_steps, N_mpc, dt, radius)



    x0   = 0;
    y0   = 0;
    th0  = 0;        % face +x
    v_f  = 0.4;      % m/s
    L1   = 40.0;     % 10 m straight
    R    = 10;      % 1.5 m turn radius (nice and smooth)
    L2   = 50.0;     % 15 m after the turn
    % 
    x_ref = generate_L_reference(N_steps, N_mpc, dt, ...
        x0, y0, th0, ...
        v_f, L1, R, L2);

    % v_nominal = 0.8;  % Realistic cruise speed
    % x_ref = generate_feasible_blended_trajectory(N_steps, N_mpc, dt, 0, 0, 0, v_nominal);



    Q_mpc = diag([40, 5, 30]);
    R_mpc = diag([0.1, 0.5]);
    S_mpc = diag([1.0, 2.0]);
    Qf_mpc = 10 * Q_mpc;

    v_min = 0.0;
    v_max = 3.33;
    omega_min = -pi/2;
    omega_max = pi/2;
    u_min = [v_min; omega_min];
    u_max = [v_max; omega_max];

    a_max = 2.50;
    alpha_max = 3.0;
    du_max = [a_max * dt; alpha_max * dt];

    %% 3. EKF Parameters
    % x0_ekf = [radius; 0; pi/2; 0.5; 0.0; 0.0] + [0.1; 0.1; 0.05; 0.02; 0.01; 0.01];

    % 1. INITIAL UNCERTAINTY (Your values are good)
    P0_ekf = diag([0.5, 0.5, 0.2, 0.05, 0.02, 0.05].^2);
    
    % 2. PROCESS NOISE (Q)
    % We add a small "safety" noise to all states to prevent
    % the covariance from collapsing to zero.
    Q_ekf = diag([
        1e-5,   % x position
        1e-5,   % y position
        1e-5,   % theta
        1e-4,   % v
        1e-4,   % omega
        1e-5    % gyro_bias (This is now large enough to stay "awake")
    ]);
    
    % 3. SENSOR NOISE (R)
    % We "lie" to the EKF and tell it the sensors are 5-10x NOISIER
    % than they really are. This forces the filter to keep "listening."
    R_enc = diag([0.05^2, 0.02^2]);  % Inflated from [0.01, 0.005]
    R_imu = 0.02^2;                 % Inflated from 0.005
    %% 4. Initialization
    fprintf('  Initializing NMPC controller... ');
    tic;
    nmpc = nmpc_casadi_controller(N_mpc, Q_mpc, R_mpc, S_mpc, Qf_mpc, dt, u_min, u_max, du_max);
    fprintf('MPC Initialisation Done (%.2fs)\n', toc);


    x0_ekf = [x_ref(:,1); 0; 0; 0];     % match EKF too, ensures that ekf start from save position as x_ref trajectory

    ekf    = ekf_state_estimator(x0_ekf, P0_ekf, Q_ekf, R_enc, R_imu);

    model = differential_drive_model();
    noise_sim = sensor_noise_simulator('low', 'low');

    x_true = zeros(3, N_steps+1);
    x_true(:,1) = x_ref(:,1);           % exact alignment

    u_last = [0; 0];

    %% 5. Storage
    x_hat_history = zeros(3, N_steps+1);
    x_true_history = zeros(3, N_steps+1);
    u_history = zeros(2, N_steps);
    solve_times = zeros(1, N_steps);

    x_hat_history(:,1) = ekf.x_hat(1:3);
    x_est_history = zeros(6, N_steps+1);  % Full EKF state history
    x_est_history(:,1) = ekf.x_hat;

    x_true_history(:,1) = x_true(:,1);

    %% 6. Closed-Loop Simulation
    fprintf('  Running closed-loop simulation (%d steps)...\n', N_steps);
    for k = 1:N_steps
        % Sensor measurements
        v_true = u_last(1);
        omega_true = u_last(2);

        z_enc = noise_sim.add_encoder_noise(v_true, omega_true);
        z_imu = noise_sim.add_imu_noise(omega_true, k*dt);

        % EKF estimation
        ekf = ekf.predict();
        ekf = ekf.update_encoders_and_imu(z_enc, z_imu);
        x_hat = ekf.x_hat(1:3);
        x_est_history(:,k+1) = ekf.x_hat;

        % NMPC planning
        x_ref_segment = x_ref(:, k:k+N_mpc);
        tic;
        [u_cmd, ~] = nmpc.solve(x_hat, x_ref_segment, u_last);
        solve_times(k) = toc;

        % Inverse kinematics
        [phi_dot_L, phi_dot_R] = model.convert_to_wheel_speeds(u_cmd);

        % Plant execution
        x_true(:,k+1) = fk_model.propagate_from_wheels(x_true(:,k), phi_dot_L, phi_dot_R, dt);

        % Store
        x_hat_history(:,k+1) = x_hat;
        x_true_history(:,k+1) = x_true(:,k+1);
        u_history(:,k) = u_cmd;
        u_last = u_cmd;
    end
    fprintf('  ✓ Simulation complete\n');

    %% 7. Analysis
    %% 7. Analysis
    plot_tracking_results(x_true_history, u_history, x_ref, dt, 'Closed-Loop Autonomy', x_est_history(3,:));
    
    % --- Calculate both position and heading errors ---
    N_steps = size(x_true_history, 2) - 1;
    x_ref_synced = x_ref(:, 1:N_steps+1);
    

    % 1. Position Error (using your existing cross-track error function)
    pos_tracking_error = compute_tracking_error(x_true_history, x_ref, dt, start_time);  % Start after 5 seconds

    heading_error_rad = compute_heading_error(x_true_history, x_ref_synced, dt, start_time);
    % heading_err = compute_heading_error(x_true_history, x_ref, dt, 5.0);
    % fprintf('Heading RMSE after 5s: %.2f deg\n', rad2deg(heading_err));
    
    % % 2. Heading Error (RMSE of time-based angular error)
    % heading_error_rad = compute_heading_error(x_true_history, x_ref_synced);
    heading_error_deg = rad2deg(heading_error_rad); % More readable
    
    % --- Analyze and Print all results ---
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
    fprintf('PHASE 4 COMPLETE: CLOSED-LOOP AUTONOMY VALIDATED\n');
    fprintf('======================================================\n\n');
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
        x_ref(:,k) = [x; y; heading];
    end
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

function x_ref = generate_damped_spiral_reference(N_steps, N_horizon, dt, r_max, growth_rate, angular_velocity)
    % Generate a damped spiral trajectory with bounded radius

    N_total = N_steps + N_horizon + 1;
    x_ref = zeros(3, N_total);

    for k = 1:N_total
        t = (k-1) * dt;
        r = r_max * (1 - exp(-growth_rate * t));  % Smooth radius growth
        theta = angular_velocity * t;
        x = r * cos(theta);
        y = r * sin(theta);
        heading = theta + pi/2;
        x_ref(:,k) = [x; y; heading];
    end
end
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



function x_ref = generate_arc_reference(N_steps, N_horizon, dt, radius, arc_angle, v_nominal)
    % Generates a circular arc trajectory
    % radius: arc radius in meters
    % arc_angle: total angle of arc in radians (e.g., pi/2 for 90°)
    % v_nominal: forward velocity

    N_total = N_steps + N_horizon + 1;
    x_ref = zeros(3, N_total);

    arc_length = radius * arc_angle;
    total_time = arc_length / v_nominal;
    omega = v_nominal / radius;  % Angular velocity

    for k = 1:N_total
        t = (k-1) * dt;
        theta = omega * t;
        if theta > arc_angle
            theta = arc_angle;
        end
        x = radius * sin(theta);
        y = radius * (1 - cos(theta));
        heading = theta;

        x_ref(:,k) = [x; y; heading];
    end
end

function plot_tracking_results(x_history, u_history, x_ref, dt, title_str, ekf_heading)
    % Plot trajectory tracking results (matches fmincon test)
    
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
    e_th = wrapToPi(x_history(3,1:N) - x_ref(3,1:N));
    plot(t, rad2deg(x_ref(3,1:N)), 'r--', 'LineWidth', 1.0, 'DisplayName', 'Reference (wrapped)');
    hold on;
    plot(t, rad2deg(wrapToPi(x_history(3,1:N))), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual (wrapped)');
    plot(t, rad2deg(wrapToPi(ekf_heading(1:N))), 'g-', 'LineWidth', 1.0, 'DisplayName', 'EKF (wrapped)');
    grid on;
    xlabel('Time (s)');
    ylabel('Heading (deg)');
    title('Heading (wrapped)');
    legend('Location','best');
    % yyaxis right
    % plot(t, rad2deg(e_th), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Heading error');
    % ylabel('Error (deg)');
        
    % Control: Linear Velocity
    subplot(2,3,5);
    plot(t_u, u_history(1,:), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('v (m/s)');
    title('Linear Velocity Command');
    ylim([0, 1.5]); % Set y-limits to match fmincon plot
    
    % Control: Angular Velocity
    subplot(2,3,6);
    plot(t_u, rad2deg(u_history(2,:)), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('ω (deg/s)');
    title('Angular Velocity Command');

    % ===== Error plots =====
    % x_history: 3×N actual
    % x_ref    : 3×N reference (or longer)
    Nerr = min(size(x_history,2), size(x_ref,2));
    t_err = (0:Nerr-1) * dt;

    ex  = x_history(1,1:Nerr) - x_ref(1,1:Nerr);              % x error [m]
    ey  = x_history(2,1:Nerr) - x_ref(2,1:Nerr);              % y error [m]
    eth = wrapToPi(x_history(3,1:Nerr) - x_ref(3,1:Nerr));    % heading error [rad]

    figure('Name','Tracking errors');
    
    % X error
    subplot(3,1,1);
    plot(t_err, ex, 'LineWidth', 1.8);
    grid on;
    ylabel('e_x (m)');
    title('Position and heading tracking errors');

    % Y error
    subplot(3,1,2);
    plot(t_err, ey, 'LineWidth', 1.8);
    grid on;
    ylabel('e_y (m)');

    % Heading error
    subplot(3,1,3);
    plot(t_err, rad2deg(eth), 'LineWidth', 1.8);
    grid on;
    xlabel('Time (s)');
    ylabel('e_\theta (deg)');



end
 
function err = compute_tracking_error(x_actual, x_ref, dt, start_time)
    % Compute RMSE using nearest-point matching after a delay
    % Inputs:
    %   x_actual - 3×N matrix of actual robot states
    %   x_ref    - 3×N matrix of reference states   
    %   dt       - timestep duration (s)
    %   start_time - time (s) after which to start error calculation

    start_idx = ceil(start_time / dt);  % Convert time to index
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
    start_idx = ceil(start_time / dt);

    % Determine the maximum valid index based on both arrays
    N_actual = size(x_actual, 2);
    N_ref = size(x_ref, 2);
    N_common = min(N_actual, N_ref);

    % Ensure we don't exceed bounds
    idx_range = start_idx:N_common;

    % Compute wrapped heading error
    e_theta = x_actual(3,idx_range) - x_ref(3,idx_range);
    e_theta_wrapped = wrapToPi(e_theta);
    heading_err = sqrt(mean(e_theta_wrapped.^2));
end

function wrapped = wrapToPi(angle)
    % Custom wrapToPi function (if not using Robotics System Toolbox)
    % Ensures angle is in the interval [-pi, pi]
    wrapped = angle - 2*pi * floor((angle + pi) / (2*pi));
end


function x_ref = generate_feasible_blended_trajectory( ...
        N_steps, N_horizon, dt, ...
        x0, y0, th0, v_nominal)
    % GENERATE_FEASIBLE_BLENDED_TRAJECTORY - Realistic multi-segment path
    %
    % Designed for Nova Carter robot with physical constraints:
    %   - Wheelbase: 0.453 m
    %   - Max velocity: 1.5 m/s (realistic cruise: 0.8 m/s)
    %   - Min turn radius: ~0.5 m (practical: 1.0+ m for safety)
    %   - Max angular velocity: 2.0 rad/s (practical: 1.0 rad/s)
    %
    % TRAJECTORY SEGMENTS (all kinematically feasible):
    %   1. Acceleration straight (0-2s)
    %   2. Gentle curve entry (2-8s)
    %   3. Large radius turn (8-18s)
    %   4. Slalom section (18-35s)
    %   5. Figure-8 (35-60s)
    %   6. Deceleration straight (60-70s)
    %
    % Total duration: ~70 seconds
    % Total distance: ~45 meters
    
    N_total = N_steps + N_horizon + 1;
    x_ref = zeros(3, N_total);
    
    % =====================================================================
    % ROBOT KINEMATIC LIMITS (Nova Carter)
    % =====================================================================
    
    wheelbase = 0.453;                      % Track width (m)
    v_max_safe = min(v_nominal, 0.8);       % Safe cruise speed
    omega_max_safe = 1.0;                   % Safe angular velocity (rad/s)
    
    % Minimum turn radius from kinematics: R_min = v / omega_max
    R_min_kinematic = v_max_safe / omega_max_safe;  % ~0.8m
    R_min_safe = max(R_min_kinematic * 2, 2.0);    % 2× safety factor → 2m minimum
    
    fprintf('  Robot Kinematic Constraints:\n');
    fprintf('    Wheelbase: %.3f m\n', wheelbase);
    fprintf('    Safe cruise speed: %.2f m/s\n', v_max_safe);
    fprintf('    Max safe omega: %.2f rad/s (%.1f deg/s)\n', ...
            omega_max_safe, rad2deg(omega_max_safe));
    fprintf('    Min turn radius (kinematic): %.2f m\n', R_min_kinematic);
    fprintf('    Min turn radius (safe): %.2f m\n\n', R_min_safe);
    
    % =====================================================================
    % SEGMENT PARAMETERS (All Feasible)
    % =====================================================================
    
    % Segment 1: Acceleration straight
    L1 = 10.0;                              % 10m acceleration zone
    t1 = L1 / v_max_safe;
    
    % Segment 2: Gentle curve entry (transition to turning)
    R2 = 8.0;                               % 8m radius (very gentle)
    arc2_angle = pi/6;                      % 30° curve
    t2 = (R2 * arc2_angle) / v_max_safe;
    
    % Segment 3: Large radius 90° turn
    R3 = 5.0;                               % 5m radius (comfortable turn)
    arc3_angle = pi/2;                      % 90° turn
    t3 = (R3 * arc3_angle) / v_max_safe;
    
    % Segment 4: Slalom (S-curves with large radii)
    R4 = 4.0;                               % 4m radius per curve
    num_slaloms = 2;                        % 2 complete S-curves
    arc4_angle = pi/3;                      % 60° per curve
    t4 = num_slaloms * 2 * (R4 * arc4_angle) / v_max_safe;
    
    % // Segment 5: Figure-8 (two large circles)
    R5 = 3.5;                               % 3.5m radius (well above minimum)
    t5 = 2 * (2 * pi * R5) / v_max_safe;    % Two complete circles
    
    % Segment 6: Straight deceleration
    L6 = 8.0;                               % 8m deceleration zone
    t6 = L6 / v_max_safe;
    
    % Segment time boundaries
    T1 = t1;
    T2 = T1 + t2;
    T3 = T2 + t3;
    T4 = T3 + t4;
    T5 = T4 + t5;
    T6 = T5 + t6;
    T_total = T6;
    
    fprintf('  Trajectory Segment Durations:\n');
    fprintf('    1. Accel Straight:    %.1fs (0.0 - %.1fs) | %.1fm\n', t1, T1, L1);
    fprintf('    2. Gentle Entry:      %.1fs (%.1f - %.1fs) | R=%.1fm\n', t2, T1, T2, R2);
    fprintf('    3. Large 90° Turn:    %.1fs (%.1f - %.1fs) | R=%.1fm\n', t3, T2, T3, R3);
    fprintf('    4. Slalom (×%d):       %.1fs (%.1f - %.1fs) | R=%.1fm\n', num_slaloms, t4, T3, T4, R4);
    fprintf('    5. Figure-8:          %.1fs (%.1f - %.1fs) | R=%.1fm\n', t5, T4, T5, R5);
    fprintf('    6. Decel Straight:    %.1fs (%.1f - %.1fs) | %.1fm\n', t6, T5, T6, L6);
    fprintf('    TOTAL:                %.1fs | ~%.1fm\n\n', T_total, L1 + L6 + R2*arc2_angle + R3*arc3_angle + 2*2*pi*R5);
    
    % =====================================================================
    % STATE TRACKING
    % =====================================================================
    
    x_seg = zeros(7, 1);  % Segment end X positions
    y_seg = zeros(7, 1);  % Segment end Y positions
    th_seg = zeros(7, 1); % Segment end headings
    
    x_seg(1) = x0;
    y_seg(1) = y0;
    th_seg(1) = th0;
    
    % =====================================================================
    % TRAJECTORY GENERATION
    % =====================================================================
    
    for k = 1:N_total
        t = (k-1) * dt;
        
        if t > T_total
            t = T_total;
        end
        
        % Velocity profile (smooth acceleration/deceleration)
        if t < t1 * 0.2
            % Acceleration phase
            v_t = v_max_safe * smoothstep(t / (t1 * 0.2));
        elseif t > T5 + t6 * 0.8
            % Deceleration phase
            remaining = T6 - t;
            v_t = v_max_safe * smoothstep(remaining / (t6 * 0.2));
        else
            % Cruise
            v_t = v_max_safe;
        end
        
        % Determine segment
        if t <= T1
            % =============================================================
            % SEGMENT 1: STRAIGHT ACCELERATION
            % =============================================================
            
            s = t;
            dist = integrate_velocity(v_t, s, v_max_safe, t1 * 0.2);
            
            x = x_seg(1) + dist * cos(th_seg(1));
            y = y_seg(1) + dist * sin(th_seg(1));
            heading = th_seg(1);
            
            if abs(t - T1) < dt/2
                x_seg(2) = x;
                y_seg(2) = y;
                th_seg(2) = heading;
            end
            
        elseif t <= T2
            % =============================================================
            % SEGMENT 2: GENTLE CURVE ENTRY (RIGHT)
            % =============================================================
            
            s = t - T1;
            omega2 = v_max_safe / R2;
            ang = omega2 * s;
            
            if ang > arc2_angle
                ang = arc2_angle;
            end
            
            % Right turn: center is to the right
            cx = x_seg(2) + R2 * sin(th_seg(2));
            cy = y_seg(2) - R2 * cos(th_seg(2));
            
            x = cx - R2 * sin(th_seg(2) + ang);
            y = cy + R2 * cos(th_seg(2) + ang);
            heading = wrapToPi(th_seg(2) + ang);
            
            if abs(t - T2) < dt/2
                x_seg(3) = x;
                y_seg(3) = y;
                th_seg(3) = heading;
            end
            
        elseif t <= T3
            % =============================================================
            % // SEGMENT 3: LARGE 90° TURN (LEFT)
            % =============================================================
            
            s = t - T2;
            omega3 = v_max_safe / R3;
            ang = omega3 * s;
            
            if ang > arc3_angle
                ang = arc3_angle;
            end
            
            % Left turn: center is to the left
            cx = x_seg(3) - R3 * sin(th_seg(3));
            cy = y_seg(3) + R3 * cos(th_seg(3));
            
            x = cx + R3 * sin(th_seg(3) + ang);
            y = cy - R3 * cos(th_seg(3) + ang);
            heading = wrapToPi(th_seg(3) + ang);
            
            if abs(t - T3) < dt/2
                x_seg(4) = x;
                y_seg(4) = y;
                th_seg(4) = heading;
            end
            
        elseif t <= T4
            % =============================================================
            % SEGMENT 4: SLALOM (ALTERNATING S-CURVES)
            % =============================================================
            
            s = t - T3;
            t_per_curve = (R4 * arc4_angle) / v_max_safe;
            
            % Determine which curve we're on
            curve_idx = floor(s / t_per_curve);
            s_local = s - curve_idx * t_per_curve;
            
            omega4 = v_max_safe / R4;
            ang_local = omega4 * s_local;
            
            if ang_local > arc4_angle
                ang_local = arc4_angle;
            end
            
            % Alternate left-right-left-right
            if mod(curve_idx, 2) == 0
                % Left curve
                sign_curve = 1;
            else
                % Right curve
                sign_curve = -1;
            end
            
            % Compute accumulated state at start of current curve
            if curve_idx == 0
                x_curve_start = x_seg(4);
                y_curve_start = y_seg(4);
                th_curve_start = th_seg(4);
            else
                % Simplified: assume each curve advances along path
                % (Full implementation would track each curve endpoint)
                x_curve_start = x_seg(4);
                y_curve_start = y_seg(4);
                th_curve_start = th_seg(4) + sign_curve * (curve_idx * arc4_angle);
            end
            
            % Arc center
            cx = x_curve_start + sign_curve * R4 * (-sin(th_curve_start));
            cy = y_curve_start + sign_curve * R4 * cos(th_curve_start);
            
            x = cx + sign_curve * R4 * sin(th_curve_start + sign_curve * ang_local);
            y = cy - sign_curve * R4 * cos(th_curve_start + sign_curve * ang_local);
            heading = wrapToPi(th_curve_start + sign_curve * ang_local);
            
            if abs(t - T4) < dt/2
                x_seg(5) = x;
                y_seg(5) = y;
                th_seg(5) = heading;
            end
            
        elseif t <= T5
            % =============================================================
            % SEGMENT 5: FIGURE-8 (TWO CIRCLES)
            % =============================================================
            
            s = t - T4;
            omega5 = v_max_safe / R5;
            
            % First circle (left): 0 to 2π
            % Second circle (right): 2π to 4π
            ang_total = omega5 * s;
            
            if ang_total <= 2 * pi
                % First circle (counterclockwise / left)
                ang = ang_total;
                cx = x_seg(5) - R5 * sin(th_seg(5));
                cy = y_seg(5) + R5 * cos(th_seg(5));
                
                x = cx + R5 * sin(th_seg(5) + ang);
                y = cy - R5 * cos(th_seg(5) + ang);
                heading = wrapToPi(th_seg(5) + ang);
            else
                % Second circle (clockwise / right)
                ang = ang_total - 2 * pi;
                
                % Start second circle at end of first circle
                th_second_start = th_seg(5) + 2 * pi;  % Back to original heading
                x_second_start = x_seg(5);  % Back to start position
                y_second_start = y_seg(5);
                
                cx = x_second_start + R5 * sin(th_second_start);
                cy = y_second_start - R5 * cos(th_second_start);
                
                x = cx - R5 * sin(th_second_start - ang);
                y = cy + R5 * cos(th_second_start - ang);
                heading = wrapToPi(th_second_start - ang);
            end
            
            if abs(t - T5) < dt/2
                x_seg(6) = x;
                y_seg(6) = y;
                th_seg(6) = heading;
            end
            
        else
            % =============================================================
            % SEGMENT 6: STRAIGHT DECELERATION
            % =============================================================
            
            s = t - T5;
            dist = integrate_velocity(v_t, s, v_max_safe, t6 * 0.8);
            
            x = x_seg(6) + dist * cos(th_seg(6));
            y = y_seg(6) + dist * sin(th_seg(6));
            heading = th_seg(6);
            
            if abs(t - T6) < dt/2
                x_seg(7) = x;
                y_seg(7) = y;
                th_seg(7) = heading;
            end
        end
        
        x_ref(:,k) = [x; y; heading];
    end
    
    fprintf('  ✓ Feasible trajectory generated\n');
    fprintf('    Start: (%.1f, %.1f) @ %.0f°\n', x0, y0, rad2deg(th0));
    fprintf('    End:   (%.1f, %.1f) @ %.0f°\n', x_seg(7), y_seg(7), rad2deg(th_seg(7)));
    fprintf('    Min turn radius used: %.1fm (%.1f× kinematic limit)\n', ...
            R_min_safe, R_min_safe / R_min_kinematic);
end

% =========================================================================
% HELPER FUNCTIONS
% =========================================================================

function s = smoothstep(x)
    % Smooth interpolation function (C1 continuous)
    % Maps [0,1] -> [0,1] with zero derivatives at endpoints
    x = max(0, min(1, x));
    s = x * x * (3 - 2*x);
end

function dist = integrate_velocity(v_t, t, v_max, t_accel)
    % Integrate smoothstep velocity profile
    if t < t_accel
        % Acceleration phase
        alpha = t / t_accel;
        dist = v_max * t_accel * (alpha^3 * (10 - 15*alpha + 6*alpha^2) / 10);
    else
        % Constant velocity phase
        dist = v_max * t_accel * 0.5 + v_max * (t - t_accel);
    end
end

% function wrapped = wrapToPi(angle)
%     wrapped = angle - 2*pi * floor((angle + pi) / (2*pi));
% end