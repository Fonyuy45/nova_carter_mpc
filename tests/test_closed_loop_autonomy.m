function test_closed_loop_autonomy()
    clc; close all;
    fprintf('\n======================================================\n');
    fprintf('PHASE 4: CLOSED-LOOP AUTONOMY — EKF + NMPC + PLANT\n');
    fprintf('======================================================\n\n');

    %% 1. Setup
    params = nova_carter_params();
    dt = params.dt;
    
    T_sim = 90.0;


    N_steps = round(T_sim / dt);
    fk_model = forward_kinematics();

    radius = 3.0;         % meters
    arc_angle = pi/2;     % 90 degrees
    v_nominal = 1.0;      % m/s


    % Reference trajectory: circular path
    radius = 5.0;
    % x_ref = generate_circular_reference(N_steps, 30, dt, radius);
    % x_ref = generate_spiral_reference(N_steps, N_mpc, dt, 0.1, 0.5);

    %% 2. NMPC Parameters
    N_mpc = 10;

    start_time = 2.0;

    % x_ref = generate_figure8_reference(N_steps, N_mpc, dt, 5.0, 1.0)
    % x_ref = generate_sine_reference(N_steps, 30, dt, 1.0, 0.05, 1.0);
    % x_ref = generate_circular_reference(N_steps, N_mpc, dt, radius);
    % x_ref = generate_spiral_reference(N_steps, N_mpc, dt, 0.1, 0.5);
    %x_ref = generate_arc_reference(N_steps, N_mpc, dt, radius, arc_angle, v_nominal)
    x0 = [5.0; 0.0; pi/2];   % for example

    % --- generate reference AFTER you know start pose ---
     x_ref = generate_spiral_reference(N_steps, N_mpc, dt, ...
                                      0.09, 0.5, ...   % growth, angular vel
                                      x0(1), x0(2), x0(3));


        
        % x0   = 5.0;     % 👈 start here
        % y0   = 0.0;
        % th0  = 0.0;     % face +x
        % v_f  = 0.4;     % 0.4 m/s
        % dy   = 2.0;     % shift up by 2 m
        % t_s  = 5.0;     % start lane change at 5 s
        % t_d  = 10.0;     % take 6 s to do it
        % 
        % x_ref = generate_lane_change_reference( ...
        %     N_steps, N_mpc, dt, ...
        %     x0, y0, th0, ...
        %     v_f, dy, t_s, t_d);

    Q_mpc = diag([10, 20, 10]);
    R_mpc = diag([0.1, 0.5]);
    S_mpc = diag([1.0, 2.0]);
    Qf_mpc = 150 * Q_mpc;

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
    x0_ekf = [radius; 0; pi/2; 0.5; 0.0; 0.0] + [0.1; 0.1; 0.05; 0.02; 0.01; 0.01];

    P0_ekf = diag([0.5, 0.5, 0.2, 0.05, 0.02, 0.05].^2);
    Q_ekf = diag([1e-6, 1e-6, 0.001, 0.05, 0.05, 1e-5]);
    R_enc = diag([0.02, 0.01].^2);
    R_imu = 0.02^2;

    %% 4. Initialization
    fprintf('  Initializing NMPC controller... ');
    tic;
    nmpc = nmpc_casadi_controller(N_mpc, Q_mpc, R_mpc, S_mpc, Qf_mpc, dt, u_min, u_max, du_max);
    fprintf('done (%.2fs)\n', toc);


    x0_ekf = [x_ref(:,1); 0; 0; 0];     % match EKF too
    ekf    = ekf_state_estimator(x0_ekf, P0_ekf, Q_ekf, R_enc, R_imu);

    % ekf = ekf_state_estimator(x0_ekf, P0_ekf, Q_ekf, R_enc, R_imu);
    model = differential_drive_model();
    noise_sim = sensor_noise_simulator('medium', 'medium');

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

% function x_ref = generate_spiral_reference(N_steps, N_horizon, dt, growth_rate, angular_velocity)
%     % Generate an outward spiral reference trajectory
%     % Robot spirals counter-clockwise with increasing radius
%     % Starts at (x0, y0) = (radius, 0) to match robot's initial state
% 
%     N_total = N_steps + N_horizon + 1;
%     x_ref = zeros(3, N_total);
% 
%     % Initial offset to match robot's starting position
%     x0 = growth_rate * 0 * cos(0);  % = 0
%     y0 = growth_rate * 0 * sin(0);  % = 0
%     x_start = growth_rate * cos(0);  % = growth_rate
%     y_start = 0;
% 
%     for k = 1:N_total
%         t = (k-1) * dt;
%         r = growth_rate * t;
%         theta = angular_velocity * t;
%         x = r * cos(theta) + (growth_rate - x_start);  % Shift to match initial X
%         y = r * sin(theta) - y_start;                  % Shift to match initial Y
%         heading = theta + pi/2;
%         x_ref(:,k) = [x; y; heading];
%     end
% end

function x_ref = generate_spiral_reference( ...
        N_steps, N_horizon, dt, growth_rate, angular_velocity, x0, y0, th0)

    N_total = N_steps + N_horizon + 1;
    x_ref   = zeros(3, N_total);

    for k = 1:N_total
        t = (k-1) * dt;

        r     = growth_rate * t;          % radius grows with time
        theta = angular_velocity * t;     % spiral angle (about origin)

        % spiral point in robot-centered polar coords
        x_sp = r * cos(theta);
        y_sp = r * sin(theta);

        % shift to desired start pose
        x     = x0 + x_sp;
        y     = y0 + y_sp;
        % heading tangent to spiral
        raw_heading = theta + th0;        % or theta + pi/2; pick ONE convention
        heading     = wrapToPi(raw_heading);
        
        x_ref(:,k) = [x; y; heading];

    end
end




function x_ref = generate_figure8_reference(N_steps, N_horizon, dt, a, v_nominal)
    % Generate a figure-8 trajectory using a lemniscate of Bernoulli
    % a: size scale (meters)
    % v_nominal: forward velocity

    N_total = N_steps + N_horizon + 1;
    x_ref = zeros(3, N_total);

    for k = 1:N_total
        t = (k-1) * dt;
        omega = v_nominal / a;  % Angular speed
        theta = omega * t;

        x = a * sin(theta);
        y = a * sin(theta) * cos(theta);
        dx_dt = a * cos(theta);
        dy_dt = a * (cos(2*theta));
        heading = atan2(dy_dt, dx_dt);

        x_ref(:,k) = [x; y; heading];
    end
end

function x_ref = generate_lane_change_reference( ...
        N_steps, N_horizon, dt, ...
        x0, y0, th0, ...
        v_forward, dy, t_start, t_duration)
%GENERATE_LANE_CHANGE_REFERENCE
%   Forward in +x, then smooth lateral shift of 'dy' meters.
%
%   - starts exactly at [x0; y0; th0]
%   - shift begins at t_start (s)
%   - shift finishes at t_start + t_duration (s)
%   - heading is tangent to the path
%
%   Inputs
%     N_steps, N_horizon, dt  - like your spiral fn
%     x0, y0, th0             - start pose
%     v_forward               - forward speed (m/s)
%     dy                      - total lateral change (m) (+ is to the left if th0=0)
%     t_start                 - when to start lane change (s)
%     t_duration              - how long to perform lane change (s)

    N_total = N_steps + N_horizon + 1;
    x_ref   = zeros(3, N_total);

    for k = 1:N_total
        t = (k-1) * dt;

        % forward motion
        x = x0 + v_forward * t;
        xdot = v_forward;

        % ----- lateral profile -----
        if t < t_start
            % before lane change
            y    = y0;
            ydot = 0;
        elseif t < t_start + t_duration
            % during lane change: smoothstep 3s^2 - 2s^3
            tau = (t - t_start) / t_duration;   % 0 -> 1
            % position shape
            sigma = 3*tau^2 - 2*tau^3;
            y = y0 + dy * sigma;

            % derivative of smoothstep
            ds_dt      = 1 / t_duration;
            sigma_dot  = (6*tau - 6*tau^2) * ds_dt;
            ydot = dy * sigma_dot;
        else
            % after lane change
            y    = y0 + dy;
            ydot = 0;
        end

        % heading along path
        path_heading = atan2(ydot, xdot);
        heading      = wrapToPi(path_heading + th0);

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

% % function err = compute_tracking_error(x_actual, x_ref)
% %     % Compute RMSE between actual and reference positions
% %     % Inputs:
% %     %   x_actual - 3×N matrix of actual robot states
% %     %   x_ref    - 3×N matrix of reference states
% %     % Output:
% %     %   err      - scalar RMSE over position [x; y]
% % 
% %     skip = 10;  % Skip first 10 steps
% %     pos_actual = x_actual(1:2,skip:end);
% %     pos_ref = x_ref(1:2,skip:end);
% % 
% %     errors_sq = (pos_actual - pos_ref).^2;
% %     mse_per_step = sum(errors_sq, 1);
% %     err = sqrt(mean(mse_per_step));
% % end
% 
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