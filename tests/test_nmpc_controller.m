%% test_nmpc_controller.m
% Test NMPC controller on simple trajectory tracking scenarios
%
% PURPOSE:
%   Validate NMPC implementation with progressively complex tests:
%   1. Straight line tracking
%   2. Circular trajectory tracking
%   3. Figure-8 trajectory
%
% SUCCESS CRITERIA:
%   ✓ Controller tracks reference trajectory
%   ✓ Respects velocity and acceleration constraints
%   ✓ Smooth control commands (no chattering)
%   ✓ Solves in real-time (<100ms per step)

function test_nmpc_controller()
    close all; clc;
    
    fprintf('\n');
    fprintf('======================================================\n');
    fprintf('PHASE 3: NMPC Controller Validation\n');
    fprintf('======================================================\n\n');
    
    %% Test 1: Straight Line Tracking
    fprintf('Test 1: Straight Line Tracking\n');
    fprintf('-------------------------------\n');
    
    % Initialize NMPC
    N = 8;              % Prediction horizon (20 steps)
    Q = diag([10, 10, 1]);    % State weights [x, y, θ]
    R = diag([0.1, 0.1]);     % Control weights [v, ω]
    S = diag([1.0, 1.0]);     %// Smoothness weights
    Q_f = 50 * Q;        % Terminal weight (strong)
    
    nmpc = nmpc_controller(N, Q, R, S, Q_f);
    params = nova_carter_params();
    
    % Simulation parameters
    T_sim = 30.0;
    dt = params.dt;
    N_steps = round(T_sim / dt);
    
    % Generate straight line reference
    x_ref = generate_straight_line_reference(N_steps, N, dt);
    
    % Initial state (slightly offset from reference)
    x_current = [0; 0.5; 0];  % Start 0.5m to the right
    u_last = [0; 0];
    
    % Storage
    x_history = zeros(3, N_steps+1);
    u_history = zeros(2, N_steps);
    x_history(:,1) = x_current;
    

    % Simulation loop
    fprintf('  Running simulation...\n');
    for k = 1:N_steps
        % Get reference trajectory segment for this time
        x_ref_segment = x_ref(:, k:k+N);
        
        % Solve NMPC
        [u_opt, ~, ~] = nmpc.solve(x_current, x_ref_segment, u_last);
    
        % Apply control (simulate robot response)
        model = differential_drive_model();
        x_current = model.dynamics_discrete(x_current, u_opt);
        
        % Store
        x_history(:,k+1) = x_current;
        u_history(:,k) = u_opt;
        u_last = u_opt;
    end
    
    fprintf('  ✓ Simulation complete\n');
    % nmpc.print_diagnostics();
    
    % Plot results
    plot_tracking_results(x_history, u_history, x_ref, dt, 'Test 1: Straight Line');
    
    % Compute tracking error
    tracking_error = compute_tracking_error(x_history, x_ref(:,1:N_steps+1));
    fprintf('  Average tracking error: %.3f cm\n', tracking_error*100);
    
    if tracking_error < 0.10
        fprintf('  ✓ PASS: Tracking error < 10 cm\n\n');
    else
        fprintf('  ✗ FAIL: Tracking error too large\n\n');
    end
    
    %% Summary
    fprintf('======================================================\n');
    fprintf('PHASE 3: Basic NMPC test complete!\n');
    fprintf('======================================================\n\n');
end

%% Helper Functions

function x_ref = generate_straight_line_reference(N_steps, N_horizon, dt)
    % Generate reference trajectory: straight line along x-axis
    %
    % INPUTS:
    %   N_steps   - Total simulation steps
    %   N_horizon - NMPC prediction horizon
    %   dt        - Time step
    %
    % OUTPUT:
    %   x_ref     - Reference trajectory (3 x N_steps+N_horizon+1)
    %               Each column: [x_ref; y_ref; θ_ref]
    
    % Total points needed (simulation + horizon)
    N_total = N_steps + N_horizon + 1;
    
    % Straight line: move at 1 m/s along x-axis
    v_ref = 1.0;
    x_ref = zeros(3, N_total);
    
    for k = 1:N_total
        t = (k-1) * dt;
        x_ref(:,k) = [v_ref * t; 0; 0];  % [x; y; θ]
    end
end

function error = compute_tracking_error(x_history, x_ref)
    % Compute average position tracking error
    %
    % INPUTS:
    %   x_history - Actual trajectory (3 x N)
    %   x_ref     - Reference trajectory (3 x N)
    %
    % OUTPUT:
    %   error     - Average Euclidean distance (m)
    
    pos_error = sqrt((x_history(1,:) - x_ref(1,:)).^2 + ...
                     (x_history(2,:) - x_ref(2,:)).^2);
    error = mean(pos_error);
end

function plot_tracking_results(x_history, u_history, x_ref, dt, title_str)
    % Plot trajectory tracking results
    %
    % INPUTS:
    %   x_history - Actual trajectory (3 x N)
    %   u_history - Control history (2 x N-1)
    %   x_ref     - Reference trajectory
    %   dt        - Time step
    %   title_str - Plot title
    
    N = size(x_history, 2);
    t = (0:N-1) * dt;
    t_u = (0:size(u_history,2)-1) * dt;
    
    figure('Name', title_str, 'Position', [100 100 1200 800]);
    
    % 2D Trajectory
    subplot(2,3,1);
    plot(x_ref(1,1:N), x_ref(2,1:N), 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(x_history(1,:), x_history(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Actual');
    plot(x_history(1,1), x_history(2,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(x_history(1,end), x_history(2,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
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
    plot(t, rad2deg(x_ref(3,1:N)), 'r--', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(x_history(3,:)), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)'); ylabel('Heading (deg)');
    title('Heading');
    
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
end