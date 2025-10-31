%% test_kinematic_model.m
% Test the kinematic model implementation with robot_state integration

clear; close all; clc;

%% Setup
params = nova_carter_params;
model = differential_drive_model();

%% Test 1: Forward simulation (straight line) with robot visualization
fprintf('Test 1: Forward motion\n');

% Create initial robot state
state = robot_state(0, 0, 0, 0);  % Start at origin
u = [1.0; 0];  % 1 m/s forward, no rotation

% Simulate for 5 seconds
T_sim = 5.0;
N_steps = round(T_sim / params.dt);

x_traj = zeros(3, N_steps+1);
x_traj(:,1) = state.toVector();

for k = 1:N_steps
    x_traj(:,k+1) = model.dynamics_discrete(x_traj(:,k), u);
end

% Plot trajectory with robot visualization
figure('Name', 'Test 1: Straight Line with Robot');
plot(x_traj(1,:), x_traj(2,:), 'b-', 'LineWidth', 2);
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Forward Motion Test');

% Visualize robot at key positions
state_start = robot_state(0,0,0).fromVector(x_traj(:,1), 0);
state_mid = robot_state(0,0,0).fromVector(x_traj(:,round(N_steps/2)), T_sim/2);
state_end = robot_state(0,0,0).fromVector(x_traj(:,end), T_sim);

state_start.plotRobot('g');
state_mid.plotRobot('b');
state_end.plotRobot('r');

legend('Trajectory', 'Start', 'Middle', 'End');

fprintf('  Final position: (%.2f, %.2f, %.2f rad)\n', ...
        state_end.x, state_end.y, state_end.theta);
fprintf('  Expected: (5.00, 0.00, 0.00 rad)\n\n');

%% Test 2: Circular motion with robot visualization
fprintf('Test 2: Circular motion\n');

% Define desired circle parameters
radius = 2.0;
center = [0; 0];

% Create initial robot state
state = robot_state(center(1) + radius, center(2), pi/2, 0);

v = 1.0;
omega = v/radius;
u = [v; omega];

% Simulate one full circle
T_sim = 2*pi/omega;
N_steps = round(T_sim / params.dt);

x_traj = zeros(3, N_steps+1);
x_traj(:,1) = state.toVector();

for k = 1:N_steps
    x_traj(:,k+1) = model.dynamics_discrete(x_traj(:,k), u);
end

% Plot
figure('Name', 'Test 2: Circle with Robot Visualization');
plot(x_traj(1,:), x_traj(2,:), 'b-', 'LineWidth', 2);
hold on;

% Plot ideal circle
theta_circle = linspace(0, 2*pi, 100);
plot(center(1) + radius*cos(theta_circle), ...
     center(2) + radius*sin(theta_circle), ...
     'r--', 'LineWidth', 1.5);

% Visualize robot at 8 equally spaced positions around circle
num_robots = 8;
for i = 1:num_robots
    idx = round((i-1) * N_steps / (num_robots-1)) + 1;
    if idx > N_steps+1, idx = N_steps+1; end
    state_i = robot_state(0,0,0).fromVector(x_traj(:,idx), (idx-1)*params.dt);
    state_i.plotRobot('r');
end

% Highlight start position
state_start = robot_state(0,0,0).fromVector(x_traj(:,1), 0);
state_start.plotRobot('g');

grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Circular Motion Test');
legend('Simulated', 'Ideal Circle', 'Location', 'best');

fprintf('  Start position: (%.2f, %.2f, %.2f rad)\n', ...
        state_start.x, state_start.y, state_start.theta);
fprintf('  Circle center: (%.2f, %.2f)\n', center(1), center(2));
fprintf('  Test complete\n\n');

%% Test 3: Wheel velocity conversion
fprintf('Test 3: Wheel velocity conversions\n');
v_test = 1.5;
omega_test = 0.5;

[v_wheels, omega_wheels] = params.chassis2wheels(v_test, omega_test);
[v_back, omega_back] = params.wheels2chassis(omega_wheels(1), omega_wheels(2));

fprintf('  Original: v=%.3f m/s, omega=%.3f rad/s\n', v_test, omega_test);
fprintf('  Wheels: omega_R=%.3f, omega_L=%.3f rad/s\n', ...
        omega_wheels(1), omega_wheels(2));
fprintf('  Reconstructed: v=%.3f m/s, omega=%.3f rad/s\n', v_back, omega_back);
fprintf('  Error: %.6f (should be near zero)\n\n', ...
        norm([v_test; omega_test] - [v_back; omega_back]));

%% Test 4: Reference trajectory
fprintf('Test 4: Reference trajectories\n');

% Circle
ref_circle = reference_trajectory('circle', 3.0, [0; 0], 1.0);

% Figure-8
ref_fig8 = reference_trajectory('figure8', 2.0, 0.5);

% Plot
figure('Name', 'Test 4: Reference Trajectories');
subplot(1,2,1);
ref_circle.plotTrajectory('b-');
grid on; axis equal;
title('Circle Reference');
xlabel('X (m)'); ylabel('Y (m)');

subplot(1,2,2);
ref_fig8.plotTrajectory('r-');
grid on; axis equal;
title('Figure-8 Reference');
xlabel('X (m)'); ylabel('Y (m)');

fprintf('  Reference trajectories generated successfully\n\n');

%% Test 5: Euler vs RK4 integration comparison with robot visualization
fprintf('Test 5: Euler vs RK4 integration comparison\n');

% Use same circle parameters from Test 2
state = robot_state(center(1) + radius, center(2), pi/2, 0);

v = 1.0;
omega = v/radius;
u = [v; omega];

T_sim = 2*pi/omega;
N_steps = round(T_sim / params.dt);

% Simulate with Euler
x_traj_euler = zeros(3, N_steps+1);
x_traj_euler(:,1) = state.toVector();

for k = 1:N_steps
    x_traj_euler(:,k+1) = model.dynamics_discrete(x_traj_euler(:,k), u);
end

% Simulate with RK4
x_traj_rk4 = zeros(3, N_steps+1);
x_traj_rk4(:,1) = state.toVector();

for k = 1:N_steps
    x_traj_rk4(:,k+1) = model.dynamics_discrete_rk4(x_traj_rk4(:,k), u);
end

% Plot comparison
figure('Name', 'Test 5: Integration Method Comparison');
hold on;
plot(x_traj_euler(1,:), x_traj_euler(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Euler');
plot(x_traj_rk4(1,:), x_traj_rk4(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'RK4');

% Plot ideal circle
theta_circle = linspace(0, 2*pi, 100);
plot(center(1) + radius*cos(theta_circle), ...
     center(2) + radius*sin(theta_circle), ...
     'r--', 'LineWidth', 1.5, 'DisplayName', 'Ideal Circle');

% Show final robot positions
state_euler = robot_state(0,0,0).fromVector(x_traj_euler(:,end), T_sim);
state_rk4 = robot_state(0,0,0).fromVector(x_traj_rk4(:,end), T_sim);

state_euler.plotRobot('b');
state_rk4.plotRobot('g');

grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)');
title('Integration Method Comparison');
legend('Location', 'best');

% Calculate errors
x0_vec = state.toVector();
euler_error = norm(x_traj_euler(:,end) - x0_vec);
rk4_error = norm(x_traj_rk4(:,end) - x0_vec);

fprintf('  Euler final position error: %.4f m\n', euler_error);
fprintf('  RK4 final position error: %.4f m\n', rk4_error);
fprintf('  Improvement: %.1fx\n\n', euler_error/rk4_error);

fprintf('All tests completed successfully!\n');