function test_closed_loop_simulation()
    clc; close all;
    fprintf('\n===== CLOSED-LOOP NMPC + EKF SIMULATION =====\n\n');

    %% Setup
    params = nova_carter_params();
    dt = params.dt;
    T_sim = 20.0;
    N_steps = round(T_sim / dt);
    N = 30;  % NMPC horizon

    % Reference trajectory (circular)
    radius = 3.0;
    x_ref = generate_circular_reference(N_steps, N, dt, radius);

    % Initial true state
    x_true = zeros(3, N_steps+1);
    x_true(:,1) = [radius; 0; pi/2];

    % Initial estimated state
    x0_est = x_true(:,1) + [0.1; 0.1; 0.05];
    P0 = diag([0.5, 0.5, 0.2].^2);
    Q = diag([1e-6, 1e-6, 0.001]);
    R_enc = diag([0.02, 0.01].^2);
    R_imu = 0.02^2;

    ekf = ekf_state_estimator(x0_est, P0, Q, R_enc, R_imu);
    nmpc = nmpc_casadi_controller(N, diag([10,10,1]), diag([0.1,0.1]), diag([1,1]), 50*diag([10,10,1]), dt, [0; -pi/2], [1; pi/2], [2*dt; 3*dt]);
    model = differential_drive_model();
    noise_sim = sensor_noise_simulator('medium', 'medium');

    %% Storage
    x_hat_history = zeros(3, N_steps+1);
    x_true_history = zeros(3, N_steps+1);
    u_history = zeros(2, N_steps);
    x_hat_history(:,1) = ekf.x_hat;
    x_true_history(:,1) = x_true(:,1);
    u_last = [0; 0];

    %% Loop
    for k = 1:N_steps
        % 1. Sensor measurements
        [v_true, omega_true] = model.get_body_velocities(x_true(:,k), u_last);
        z_enc = noise_sim.add_encoder_noise(v_true, omega_true);
        z_imu = noise_sim.add_imu_noise(omega_true, k*dt);

        % 2. EKF estimation
        ekf = ekf.predict();  % or pass wheel speeds if needed
        ekf = ekf.update_encoders_and_imu(z_enc, z_imu);
        x_hat = ekf.x_hat;

        % 3. NMPC planning
        x_ref_segment = x_ref(:, k:k+N);
        [u_cmd, ~] = nmpc.solve(x_hat, x_ref_segment, u_last);

        % 4. Inverse kinematics
        [phi_dot_L, phi_dot_R] = model.convert_to_wheel_speeds(u_cmd);

        % 5. Plant execution
        x_true(:,k+1) = forward_kinematics.propagate_from_wheels(x_true(:,k), phi_dot_L, phi_dot_R, dt);

        % 6. Store
        x_hat_history(:,k+1) = x_hat;
        x_true_history(:,k+1) = x_true(:,k+1);
        u_history(:,k) = u_cmd;
        u_last = u_cmd;
    end

    %% Analysis
    plot_closed_loop_results(x_true_history, x_hat_history, x_ref, u_history, dt);
end