%% test_nmpc_casadi_controller.m
% Test CasADi-based NMPC controller on simple trajectory tracking scenarios
%
% PURPOSE:
%   Validate NMPC implementation using CasADi + IPOPT
%   1. Straight line tracking
%
% SUCCESS CRITERIA:
%   ✓ Controller tracks reference trajectory
%   ✓ Respects velocity and acceleration constraints
%   ✓ Smooth control commands (no chattering)
%   ✓ Solves in real-time (<100ms per step)
function test_nmpc_casadi_controller()
    close all; clc;
    fprintf('\n');
    fprintf('======================================================\n');
    fprintf('PHASE 3: CasADi NMPC Controller Validation\n');
    fprintf('======================================================\n\n');

    %% Test 1: Straight Line Tracking
    fprintf('Test 1: Straight Line Tracking (CasADi)\n');
    fprintf('----------------------------------------\n');
    
    % ---------------------------------------------------------------------
    % 1. Controller & Simulation Setup
    % ---------------------------------------------------------------------
    
    % NMPC parameters
    N = 29;                          % Prediction horizon (steps)
    Q = diag([10, 10, 1]);           % State weights [x, y, θ]
    R = diag([0.1, 0.1]);            % Control weights [v, ω]
    S = diag([1.0, 1.0]);            % Smoothness weights (rate penalty)
    Qf = 50 * Q;                     % Terminal weight (high to ensure final state)
    
    % Robot parameters
    params = nova_carter_params();
    dt = params.dt;                  % Time step (e.g., 0.1s)
    
    % Control limits (Box Constraints)
    v_min = 0.0;
    v_max = 1.0;
    omega_min = -pi/2;
    omega_max = pi/2;
    u_min = [v_min; omega_min];
    u_max = [v_max; omega_max];
    
    % Rate limits (Linear Constraints)
    a_max = 2.0;        % Max linear acceleration (m/s²)
    alpha_max = 3.0;    % Max angular acceleration (rad/s²)
    % Convert acceleration to per-step change
    du_max = [a_max * dt; alpha_max * dt];
    
    % ---------------------------------------------------------------------
    % 2. Initialize Controller and Simulation
    % ---------------------------------------------------------------------
    
    % Initialize CasADi NMPC controller
    % This is the "Build Once" step. It is *expected* to be slow (0.5-2s)
    % as it builds the symbolic graph and creates the solver.
    fprintf('  Initializing NMPC (building solver)...');
    tic;
    nmpc = nmpc_casadi_controller(N, Q, R, S, Qf, dt, u_min, u_max, du_max);
    fprintf(' done (%.2fs)\n', toc);
    
    % Initialize the robot's simulation model (create ONCE)
    model = differential_drive_model();
    
    % Simulation parameters
    T_sim = 30.0;                  % Total simulation time
    N_steps = round(T_sim / dt);   % Total number of simulation steps
    
    % Generate reference trajectory
    % We need N_steps for the simulation, plus N for the final horizon
    x_ref = generate_straight_line_reference(N_steps, N, dt);
    
    % Initial state
    % Start the robot 0.5m off the reference line (y=0) to test tracking
    x_current = [0; 0.5; 0];
    u_last = [0; 0];               % Assume last command was "stop"
    
    % Storage for results
    x_history = zeros(3, N_steps+1);  % Store all states
    u_history = zeros(2, N_steps);    % Store all commands
    solve_times = zeros(1, N_steps);  % Store solver performance
    x_history(:,1) = x_current;
    
    % ---------------------------------------------------------------------
    % 3. Run Simulation Loop
    % ---------------------------------------------------------------------
    
    fprintf('  Running simulation (%d steps)...\n', N_steps);
    for k = 1:N_steps
        % Get the reference segment for the controller
        % This is the reference from the current time 'k' to 'k+N'
        x_ref_segment = x_ref(:, k:k+N);
        
        % --- This is the "Solve Many" step ---
        % It calls the pre-built solver and should be very fast.
        tic;
        [u_cmd, ~] = nmpc.solve(x_current, x_ref_segment, u_last);
        solve_times(k) = toc;
        
        % Simulate the robot's response to the optimal command
        x_current = model.dynamics_discrete(x_current, u_cmd);
        
        % Store history
        x_history(:,k+1) = x_current;
        u_history(:,k) = u_cmd;
        u_last = u_cmd;
    end
    fprintf('  ✓ Simulation complete\n');
    
    % ---------------------------------------------------------------------
    % 4. Analyze Results
    % ---------------------------------------------------------------------
    
    % Plot tracking and control effort
    plot_tracking_results(x_history, u_history, x_ref, dt, 'CasADi NMPC: Straight Line');
    
    % Analyze tracking error
    % We compare the history to the first N_steps+1 points of the reference
    tracking_error = compute_tracking_error(x_history, x_ref(:,1:N_steps+1));
    fprintf('  Average tracking error: %.3f cm\n', tracking_error*100);
    if tracking_error < 0.10
        fprintf('  ✓ PASS: Tracking error < 10 cm\n');
    else
        fprintf('  ✗ FAIL: Tracking error too large\n');
    end
    
    % Analyze solve times
    avg_solve_time_ms = mean(solve_times) * 1000;
    max_solve_time_ms = max(solve_times) * 1000;
    fprintf('  Avg solve time: %.2f ms\n', avg_solve_time_ms);
    fprintf('  Max solve time: %.2f ms\n', max_solve_time_ms);
    
    % Pass/Fail based on real-time capability (e.g., must be faster than dt)
    if max_solve_time_ms < dt * 1000
        fprintf('  ✓ PASS: Solver is real-time capable (Max < %.0f ms)\n\n', dt*1000);
    else
        fprintf('  ✗ FAIL: Solver is NOT real-time capable (Max > %.0f ms)\n\n', dt*1000);
    end
    
    %% Summary
    fprintf('======================================================\n');
    fprintf('PHASE 3: CasADi NMPC test complete!\n');
    fprintf('======================================================\n\n');
end

% -------------------------------------------------------------------------
% Helper Functions
% -------------------------------------------------------------------------

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
    
    % Total points needed (simulation steps + one full horizon)
    N_total = N_steps + N_horizon + 1;
    
    % Straight line: move at 0.5 m/s along x-axis
    v_ref = 0.5;
    x_ref = zeros(3, N_total);
    
    for k = 1:N_total
        t = (k-1) * dt;
        x_ref(:,k) = [v_ref * t; 0; 0];  % [x_pos; y_pos; theta]
    end
end 

function err = compute_tracking_error(x_history, x_ref)
    % Compute the average position error (RMSE)
    
    % Get only the position [x; y]
    pos_hist = x_history(1:2,:);
    pos_ref = x_ref(1:2,:);
    
    % Calculate squared errors
    errors_sq = (pos_hist - pos_ref).^2;
    
    % Calculate mean squared error for each step
    mse_per_step = sum(errors_sq, 1);
    
    % Calculate overall root mean squared error
    err = sqrt(mean(mse_per_step));
end

function plot_tracking_results(x_history, u_history, x_ref, dt, title_str)
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
    plot(t, rad2deg(x_ref(3,1:N)), 'r--', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(unwrap(x_history(3,:))), 'b-', 'LineWidth', 2); % Use unwrap for clean plot
    grid on;
    xlabel('Time (s)'); ylabel('Heading (deg)');
    title('Heading');
    
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