function test_closed_loop_autonomy()
    clc; close all;
    fprintf('\n======================================================\n');
    fprintf('PHASE 4: CLOSED-LOOP AUTONOMY — EKF + NMPC + PLANT\n');
    fprintf('======================================================\n\n');

    %% 1. Setup
    params = nova_carter_params();
    dt = params.dt;
    T_sim = 60.0;
    N_steps = round(T_sim / dt);
    fk_model = forward_kinematics();


    % Reference trajectory: circular path
    radius = 5.0;
    x_ref = generate_circular_reference(N_steps, 30, dt, radius);
    % x_ref = generate_spiral_reference(N_steps, N_mpc, dt, 0.1, 0.5);

    %% 2. NMPC Parameters
    N_mpc = 20;

    x_ref = generate_figure8_reference(N_steps, N_mpc, dt, 5.0, 1.0)
    % x_ref = generate_sine_reference(N_steps, 30, dt, 1.0, 0.05, 1.0);
    % x_ref = generate_circular_reference(N_steps, N_mpc, dt, radius);
    x_ref = generate_spiral_reference(N_steps, N_mpc, dt, 0.05, 0.5)

    Q_mpc = diag([10, 10, 1]);
    R_mpc = diag([0.1, 0.5]);
    S_mpc = diag([1.0, 2]);
    Qf_mpc = 100 * Q_mpc;

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

    ekf = ekf_state_estimator(x0_ekf, P0_ekf, Q_ekf, R_enc, R_imu);
    model = differential_drive_model();
    noise_sim = sensor_noise_simulator('medium', 'medium');

    x_true = zeros(3, N_steps+1);
    x_true(:,1) = [radius; 0; pi/2];
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
    plot_tracking_results(x_true_history, u_history, x_ref, dt, 'Closed-Loop Autonomy', x_est_history(3,:));
    tracking_error = compute_tracking_error(x_true_history, x_ref(:,1:N_steps+1));
    avg_solve_time_ms = mean(solve_times) * 1000;
    max_solve_time_ms = max(solve_times) * 1000;

    fprintf('  Final tracking error: %.2f cm\n', tracking_error * 100);
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
    % Starts at (x0, y0) = (radius, 0) to match robot's initial state

    N_total = N_steps + N_horizon + 1;
    x_ref = zeros(3, N_total);

    % Initial offset to match robot's starting position
    x0 = growth_rate * 0 * cos(0);  % = 0
    y0 = growth_rate * 0 * sin(0);  % = 0
    x_start = growth_rate * cos(0);  % = growth_rate
    y_start = 0;

    for k = 1:N_total
        t = (k-1) * dt;
        r = growth_rate * t;
        theta = angular_velocity * t;
        x = r * cos(theta) + (growth_rate - x_start);  % Shift to match initial X
        y = r * sin(theta) - y_start;                  % Shift to match initial Y
        heading = theta + pi/2;
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
    % Heading
    subplot(2,3,4);
    plot(t, rad2deg(x_ref(3,1:N)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Reference'); hold on;
    plot(t, rad2deg(unwrap(x_history(3,:))), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual'); % From plant
    plot(t, rad2deg(unwrap(ekf_heading(1:N))), 'g-', 'LineWidth', 1.5, 'DisplayName', 'EKF Estimate'); % From estimator
    grid on;
    xlabel('Time (s)'); ylabel('Heading (deg)');
    title('Heading');
    legend('Location', 'best');
        
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

end

% function err = compute_tracking_error(x_actual, x_ref)
%     % Compute RMSE between actual and reference positions
%     % Inputs:
%     %   x_actual - 3×N matrix of actual robot states
%     %   x_ref    - 3×N matrix of reference states
%     % Output:
%     %   err      - scalar RMSE over position [x; y]
% 
%     skip = 10;  % Skip first 10 steps
%     pos_actual = x_actual(1:2,skip:end);
%     pos_ref = x_ref(1:2,skip:end);
% 
%     errors_sq = (pos_actual - pos_ref).^2;
%     mse_per_step = sum(errors_sq, 1);
%     err = sqrt(mean(mse_per_step));
% end

function err = compute_tracking_error(x_actual, x_ref)
    % Compute RMSE using nearest-point matching
    N = size(x_actual, 2);
    errors_sq = zeros(1, N);

    for k = 1:N
        actual_pos = x_actual(1:2, k);
        ref_pos_all = x_ref(1:2, :);
        dists = vecnorm(ref_pos_all - actual_pos, 2, 1);
        min_dist = min(dists);
        errors_sq(k) = min_dist^2;
    end

    err = sqrt(mean(errors_sq));
end