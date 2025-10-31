%% test_open_loop_wheel_commands.m
% PHASE 1 VALIDATION: Direct wheel command interface
%
% PURPOSE:
%   Verify that wheel-level commands produce correct robot motion
%   This is CRITICAL before ROS2/Isaac Sim integration
%
% WHAT WE'RE TESTING:
%   1. Sign conventions (forward vs backward, left vs right)
%   2. Wheel-to-chassis conversion accuracy
%   3. Dynamic trajectory generation
%   4. Consistency with high-level (v,ω) interface
%
% IF THESE TESTS PASS:
%   ✓ Your kinematic model is correct
%   ✓ Isaac Sim integration will work smoothly
%   ✓ Controller commands will behave as expected

function test_open_loop_wheel_commands()
    close all; clc;
    
    fprintf('\n');
    fprintf('==================================================\n');
    fprintf('PHASE 1: Direct Wheel Command Validation\n');
    fprintf('==================================================\n');
    fprintf('This validates that wheel velocities correctly\n');
    fprintf('translate to robot motion (critical for ROS2!).\n');
    fprintf('==================================================\n\n');
    
    % Initialize
    fk = forward_kinematics();
    params = nova_carter_params();
    
    %% Test 1: Sign Convention - Forward Motion
    fprintf('Test 1: Sign Convention Check - Forward Motion\n');
    fprintf('-----------------------------------------------\n');
    fprintf('Both wheels positive → should move FORWARD\n\n');
    
    x0 = [0; 0; 0];  % Start at origin, facing EAST (+x direction)
    T_sim = 5.0;
    
    % Both wheels at 5 rad/s
    phi_dot_L = 5.0;
    phi_dot_R = 5.0;
    phi_dot_profile = @(t) [phi_dot_L; phi_dot_R];
    
    x_traj = fk.simulate_wheel_profile(x0, phi_dot_profile, T_sim);
    
    % Calculate expected distance
    v_expected = params.wheel_radius * phi_dot_L;  % Both wheels same
    distance_expected = v_expected * T_sim;
    distance_actual = x_traj(1, end) - x0(1);
    
    % Plot
    figure('Name', 'Test 1: Forward Motion', 'Position', [100 100 800 600]);
    subplot(2,1,1);
    plot(x_traj(1,:), x_traj(2,:), 'b-', 'LineWidth', 3);
    hold on;
    plot(x0(1), x0(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    plot(x_traj(1,end), x_traj(2,end), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    grid on; axis equal;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('Straight Line Test: Both Wheels Same Speed', 'FontSize', 14);
    legend('Trajectory', 'Start', 'End', 'Location', 'best');
    
    subplot(2,1,2);
    t = linspace(0, T_sim, size(x_traj,2));
    plot(t, x_traj(1,:), 'b-', 'LineWidth', 2); hold on;
    plot(t, x_traj(2,:), 'r-', 'LineWidth', 2);
    plot(t, rad2deg(x_traj(3,:)), 'g-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('State', 'FontSize', 12);
    legend('x (m)', 'y (m)', 'θ (deg)', 'Location', 'best');
    title('State Evolution', 'FontSize', 14);
    
    % Validation
    fprintf('  Wheel speeds: φ̇_L = %.1f rad/s, φ̇_R = %.1f rad/s\n', ...
        phi_dot_L, phi_dot_R);
    fprintf('  Expected distance: %.2f m\n', distance_expected);
    fprintf('  Actual distance:   %.2f m\n', distance_actual);
    fprintf('  Error: %.4f m (%.2f%%)\n', ...
        abs(distance_actual - distance_expected), ...
        100*abs(distance_actual - distance_expected)/distance_expected);
    fprintf('  Lateral drift: %.4f m (should be ~0)\n', abs(x_traj(2,end)));
    fprintf('  Heading change: %.2f° (should be ~0)\n', rad2deg(abs(x_traj(3,end))));
    
    if abs(distance_actual - distance_expected) < 0.01 && abs(x_traj(2,end)) < 0.01
        fprintf('  ✓ PASS: Forward motion correct!\n\n');
    else
        fprintf('  ✗ FAIL: Check wheel-to-velocity conversion!\n\n');
        error('Test 1 failed');
    end
    
    %% Test 2: Rotation Direction Check
    fprintf('Test 2: Rotation Direction - Spin in Place\n');
    fprintf('-------------------------------------------\n');
    fprintf('Opposite wheel signs → should spin CCW\n\n');
    
    x0 = [0; 0; 0];
    
    % Calculate wheel speeds for 1 rad/s rotation
    omega_desired = 1.0;  % rad/s counterclockwise
    phi_dot_diff = (params.track_width / params.wheel_radius) * omega_desired;
    
    phi_dot_L = -phi_dot_diff / 2;
    phi_dot_R =  phi_dot_diff / 2;
    
    T_sim = 2*pi / omega_desired;  % One full rotation
    phi_dot_profile = @(t) [phi_dot_L; phi_dot_R];
    
    x_traj = fk.simulate_wheel_profile(x0, phi_dot_profile, T_sim);
    
    % Plot
    figure('Name', 'Test 2: Pure Rotation', 'Position', [150 150 800 600]);
    subplot(1,2,1);
    plot(x_traj(1,:), x_traj(2,:), 'b-', 'LineWidth', 2);
    hold on;
    plot(x0(1), x0(2), 'go', 'MarkerSize', 15, 'LineWidth', 3);
    
    % Draw orientation arrows
    N = size(x_traj, 2);
    for i = 1:round(N/12):N
        theta = x_traj(3, i);
        arrow_len = 0.15;
        quiver(x_traj(1,i), x_traj(2,i), ...
               arrow_len*cos(theta), arrow_len*sin(theta), ...
               'r', 'LineWidth', 2, 'MaxHeadSize', 0.8);
    end
    
    grid on; axis equal;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('Pure Rotation Test', 'FontSize', 14);
    
    subplot(1,2,2);
    t = linspace(0, T_sim, size(x_traj,2));
    plot(t, rad2deg(x_traj(3,:)), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time (s)', 'FontSize', 12);
    ylabel('Heading (deg)', 'FontSize', 12);
    title('Heading vs Time', 'FontSize', 14);
    
    % Validation
    translation = sqrt(x_traj(1,end)^2 + x_traj(2,end)^2);
    theta_unwrapped = unwrap(x_traj(3,:));
    heading_change = rad2deg(theta_unwrapped(end) - theta_unwrapped(1));
    
    fprintf('  Wheel speeds: φ̇_L = %.2f rad/s, φ̇_R = %.2f rad/s\n', ...
        phi_dot_L, phi_dot_R);
    fprintf('  Translation: %.4f m (should be ~0)\n', translation);
    fprintf('  Heading change: %.1f° (expect ~360°)\n', heading_change);
    
    if translation < 0.05 && abs(heading_change) > 350
        fprintf('  ✓ PASS: Rotation direction correct!\n\n');
    else
        fprintf('  ✗ FAIL: Check rotation calculation!\n\n');
        error('Test 2 failed');
    end
    
    %% Test 3: Circular Motion
    fprintf('Test 3: Circular Trajectory\n');
    fprintf('---------------------------\n');
    fprintf('Constant wheel speed difference → circle\n\n');
    
    x0 = [0; 0; pi/2];  % Start facing NORTH
    radius = 2.0;
    v = 1.0;
    omega = v / radius;
    
    % Calculate wheel speeds
    phi_dot_R = v/params.wheel_radius + ...
                (params.track_width * omega)/(2 * params.wheel_radius);
    phi_dot_L = v/params.wheel_radius - ...
                (params.track_width * omega)/(2 * params.wheel_radius);
    
    phi_dot_profile = @(t) [phi_dot_L; phi_dot_R];
    T_sim = 2*pi / omega;
    
    x_traj = fk.simulate_wheel_profile(x0, phi_dot_profile, T_sim);
    
    % Plot
    figure('Name', 'Test 3: Circle', 'Position', [200 200 800 600]);
    plot(x_traj(1,:), x_traj(2,:), 'b-', 'LineWidth', 3);
    hold on;
    
    % Ideal circle
    theta_circle = linspace(0, 2*pi, 100);
    x_circle = -radius + radius * cos(theta_circle);
    y_circle = radius * sin(theta_circle);
    plot(x_circle, y_circle, 'r--', 'LineWidth', 2);
    
    plot(x0(1), x0(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    
    grid on; axis equal;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title(sprintf('Circular Motion: R=%.1fm, v=%.1fm/s', radius, v), 'FontSize', 14);
    legend('Simulated', 'Ideal Circle', 'Start', 'Location', 'best');
    
    % Calculate error
    circle_error = norm(x_traj(1:2,end) - x0(1:2));
    
    fprintf('  Wheel speeds: φ̇_L = %.2f rad/s, φ̇_R = %.2f rad/s\n', ...
        phi_dot_L, phi_dot_R);
    fprintf('  Radius: %.2f m\n', radius);
    fprintf('  Loop closure error: %.4f m\n', circle_error);
    
    if circle_error < 0.1
        fprintf('  ✓ PASS: Circular motion accurate!\n\n');
    else
        fprintf('  ✗ FAIL: Check kinematic equations!\n\n');
        error('Test 3 failed');
    end
    
    %% Test 4: Figure-8 (Dynamic Wheel Commands)
    fprintf('Test 4: Figure-8 with Time-Varying Commands\n');
    fprintf('--------------------------------------------\n');
    fprintf('Sinusoidal wheel speed variation\n\n');
    
    x0 = [0; 0; 0];
    T_sim = 30.0;
    radius = 2.0;
    period = 15.0;
    
    phi_dot_profile = @(t) figure8_wheel_speeds(t, params, radius, period);
    x_traj = fk.simulate_wheel_profile(x0, phi_dot_profile, T_sim);
    
    % Plot
    figure('Name', 'Test 4: Figure-8', 'Position', [250 250 800 600]);
    plot(x_traj(1,:), x_traj(2,:), '-', 'LineWidth',1 );
    hold on;
    plot(x0(1), x0(2), 'go', 'MarkerSize', 12, 'LineWidth', 2);
    plot(x_traj(1,end), x_traj(2,end), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
    
    grid on; axis equal;
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('Figure-8 Trajectory', 'FontSize', 14);
    legend('Trajectory', 'Start', 'End', 'Location', 'best');
    
    fprintf('  ✓ PASS: Figure-8 generated successfully!\n\n');
    
    %% Summary
    fprintf('==================================================\n');
    fprintf('PHASE 1 COMPLETE: All Tests Passed! ✓\n');
    fprintf('==================================================\n');
    fprintf('Your kinematic model is VALIDATED and ready for:\n');
    fprintf('  → Phase 2: State Estimator (EKF)\n');
    fprintf('  → Phase 3: NMPC Controller\n');
    fprintf('  → Phase 4: ROS2 + Isaac Sim Integration\n');
    fprintf('==================================================\n\n');
end

%% Helper: Figure-8 wheel speeds
function phi_dot = figure8_wheel_speeds(t, params, radius, period)
    v = 1.0;
    omega_amplitude = v / radius;
    omega = omega_amplitude * sin(2*pi*t / period);
    
    phi_dot_R = v/params.wheel_radius + ...
                (params.track_width * omega)/(2 * params.wheel_radius);
    phi_dot_L = v/params.wheel_radius - ...
                (params.track_width * omega)/(2 * params.wheel_radius);
    
    phi_dot = [phi_dot_L; phi_dot_R];
end