%% test_parameters.m
% Validate Nova Carter parameters for physical consistency and reasonableness
% This catches configuration errors early before they cause weird simulation bugs

function test_parameters()
    fprintf('\n========================================\n');
    fprintf('Testing Nova Carter Parameters\n');
    fprintf('========================================\n\n');
    
    params = nova_carter_params;
    all_passed = true;
    
    %% Test 1: Physical Dimensions
    fprintf('Test 1: Physical Dimensions\n');
    test_passed = true;
    
    % Check wheel radius is positive and reasonable
    if params.wheel_radius <= 0
        fprintf('  ✗ FAIL: Wheel radius must be positive\n');
        test_passed = false;
    elseif params.wheel_radius < 0.05 || params.wheel_radius > 0.5
        fprintf('  ⚠ WARNING: Wheel radius %.3fm seems unusual (expected 0.05-0.5m)\n', ...
                params.wheel_radius);
    else
        fprintf('  ✓ Wheel radius: %.3f m\n', params.wheel_radius);
    end
    
    % Check track width
    if params.track_width <= 0
        fprintf('  ✗ FAIL: Track width must be positive\n');
        test_passed = false;
    elseif params.track_width < 0.2 || params.track_width > 1.0
        fprintf('  ⚠ WARNING: Track width %.3fm seems unusual (expected 0.2-1.0m)\n', ...
                params.track_width);
    else
        fprintf('  ✓ Track width: %.3f m\n', params.track_width);
    end
    
    % Check half_track consistency
    if abs(params.half_track - params.track_width/2) > 1e-6
        fprintf('  ✗ FAIL: half_track (%.3f) != track_width/2 (%.3f)\n', ...
                params.half_track, params.track_width/2);
        test_passed = false;
    else
        fprintf('  ✓ Half track: %.3f m\n', params.half_track);
    end
    
    % Check robot dimensions
    if params.length <= 0 || params.width <= 0 || params.height <= 0
        fprintf('  ✗ FAIL: Robot dimensions must be positive\n');
        test_passed = false;
    else
        fprintf('  ✓ Robot dimensions: %.3f × %.3f × %.3f m (L×W×H)\n', ...
                params.length, params.width, params.height);
    end
    
    % Width should be at least as large as track width
    if params.width < params.track_width
        fprintf('  ✗ FAIL: Robot width (%.3f) < track width (%.3f)\n', ...
                params.width, params.track_width);
        test_passed = false;
    end
    
    printTestResult('Physical Dimensions', test_passed);
    all_passed = all_passed && test_passed;
    
    %% Test 2: Velocity Limits
    fprintf('\nTest 2: Velocity Limits\n');
    test_passed = true;
    
    % Forward velocity limits
    if params.v_max <= 0
        fprintf('  ✗ FAIL: v_max must be positive\n');
        test_passed = false;
    elseif params.v_max > 10
        fprintf('  ⚠ WARNING: v_max = %.2f m/s is very high for indoor robot\n', ...
                params.v_max);
    else
        fprintf('  ✓ Max forward velocity: %.2f m/s (%.1f km/h)\n', ...
                params.v_max, params.v_max * 3.6);
    end
    
    if params.v_min > 0
        fprintf('  ✗ FAIL: v_min should be ≤ 0 (allows reverse)\n');
        test_passed = false;
    else
        fprintf('  ✓ Min forward velocity: %.2f m/s\n', params.v_min);
    end
    
    if params.v_min >= params.v_max
        fprintf('  ✗ FAIL: v_min (%.2f) >= v_max (%.2f)\n', ...
                params.v_min, params.v_max);
        test_passed = false;
    end
    
    % Angular velocity limits
    if params.omega_max <= 0
        fprintf('  ✗ FAIL: omega_max must be positive\n');
        test_passed = false;
    else
        fprintf('  ✓ Max angular velocity: %.2f rad/s (%.1f deg/s)\n', ...
                params.omega_max, rad2deg(params.omega_max));
    end
    
    if params.omega_min >= params.omega_max
        fprintf('  ✗ FAIL: omega_min (%.2f) >= omega_max (%.2f)\n', ...
                params.omega_min, params.omega_max);
        test_passed = false;
    end
    
    % Symmetry check (should be able to turn equally in both directions)
    if abs(params.omega_min + params.omega_max) > 1e-6
        fprintf('  ⚠ WARNING: Angular limits not symmetric (min=%.2f, max=%.2f)\n', ...
                params.omega_min, params.omega_max);
    end
    
    printTestResult('Velocity Limits', test_passed);
    all_passed = all_passed && test_passed;
    
    %% Test 3: Acceleration Limits
    fprintf('\nTest 3: Acceleration Limits\n');
    test_passed = true;
    
    if params.a_max <= 0
        fprintf('  ✗ FAIL: a_max must be positive\n');
        test_passed = false;
    elseif params.a_max > 5
        fprintf('  ⚠ WARNING: a_max = %.2f m/s² is very high\n', params.a_max);
    else
        fprintf('  ✓ Max linear acceleration: %.2f m/s²\n', params.a_max);
    end
    
    if params.alpha_max <= 0
        fprintf('  ✗ FAIL: alpha_max must be positive\n');
        test_passed = false;
    elseif params.alpha_max > 10
        fprintf('  ⚠ WARNING: alpha_max = %.2f rad/s² is very high\n', ...
                params.alpha_max);
    else
        fprintf('  ✓ Max angular acceleration: %.2f rad/s² (%.1f deg/s²)\n', ...
                params.alpha_max, rad2deg(params.alpha_max));
    end
    
    % Acceleration consistency check: time to reach max velocity
    t_accel = params.v_max / params.a_max;
    if t_accel < 0.1
        fprintf('  ⚠ WARNING: Very short acceleration time (%.2f s)\n', t_accel);
    else
        fprintf('  ✓ Time to max velocity: %.2f s\n', t_accel);
    end
    
    printTestResult('Acceleration Limits', test_passed);
    all_passed = all_passed && test_passed;
    
    %% Test 4: MPC Parameters
    fprintf('\nTest 4: MPC Parameters\n');
    test_passed = true;
    
    % Sampling time
    if params.dt <= 0
        fprintf('  ✗ FAIL: dt must be positive\n');
        test_passed = false;
    elseif params.dt > 0.1
        fprintf('  ⚠ WARNING: dt = %.3f s is quite large (>100ms)\n', params.dt);
    else
        fprintf('  ✓ Sampling time: %.3f s (%.0f Hz)\n', ...
                params.dt, 1/params.dt);
    end
    
    % Prediction horizon
    if params.N <= 0 || mod(params.N, 1) ~= 0
        fprintf('  ✗ FAIL: N must be positive integer\n');
        test_passed = false;
    elseif params.N < 5
        fprintf('  ⚠ WARNING: N = %d is very short\n', params.N);
    elseif params.N > 100
        fprintf('  ⚠ WARNING: N = %d is very long (computational cost)\n', params.N);
    else
        fprintf('  ✓ Prediction horizon: %d steps (%.2f s)\n', ...
                params.N, params.N * params.dt);
    end
    
    % Check if horizon is long enough to stop
    stopping_distance = 0.5 * params.v_max^2 / params.a_max;
    horizon_distance = params.v_max * params.N * params.dt;
    if horizon_distance < stopping_distance
        fprintf('  ⚠ WARNING: Horizon (%.2fm) < stopping distance (%.2fm)\n', ...
                horizon_distance, stopping_distance);
    else
        fprintf('  ✓ Horizon covers stopping distance\n');
    end
    
    % Cost weights
    if params.Q_pos < 0 || params.Q_theta < 0
        fprintf('  ✗ FAIL: Q weights must be non-negative\n');
        test_passed = false;
    else
        fprintf('  ✓ State weights: Q_pos=%.2f, Q_theta=%.2f\n', ...
                params.Q_pos, params.Q_theta);
    end
    
    if params.R_v < 0 || params.R_omega < 0
        fprintf('  ✗ FAIL: R weights must be non-negative\n');
        test_passed = false;
    else
        fprintf('  ✓ Input weights: R_v=%.2f, R_omega=%.2f\n', ...
                params.R_v, params.R_omega);
    end
    
    if params.S_v < 0 || params.S_omega < 0
        fprintf('  ✗ FAIL: S weights must be non-negative\n');
        test_passed = false;
    else
        fprintf('  ✓ Rate weights: S_v=%.2f, S_omega=%.2f\n', ...
                params.S_v, params.S_omega);
    end
    
    printTestResult('MPC Parameters', test_passed);
    all_passed = all_passed && test_passed;
    
    %% Test 5: Physical Mass and Payload
    fprintf('\nTest 5: Mass and Payload\n');
    test_passed = true;
    
    if params.robot_mass <= 0
        fprintf('  ✗ FAIL: robot_mass must be positive\n');
        test_passed = false;
    else
        fprintf('  ✓ Robot mass: %.1f kg\n', params.robot_mass);
    end
    
    if params.max_payload <= 0
        fprintf('  ✗ FAIL: max_payload must be positive\n');
        test_passed = false;
    else
        fprintf('  ✓ Max payload: %.1f kg\n', params.max_payload);
    end
    
    total_mass = params.robot_mass + params.max_payload;
    fprintf('  ✓ Total mass (loaded): %.1f kg\n', total_mass);
    
    % Sanity check on total mass
    if total_mass > 200
        fprintf('  ⚠ WARNING: Total mass (%.1f kg) is very high\n', total_mass);
    end
    
    printTestResult('Mass and Payload', test_passed);
    all_passed = all_passed && test_passed;
    
    %% Test 6: Kinematic Consistency
    fprintf('\nTest 6: Kinematic Consistency\n');
    test_passed = true;
    
    % Minimum turning radius at max speed
    % omega = v / R  =>  R = v / omega
    min_turn_radius = params.v_max / params.omega_max;
    fprintf('  ✓ Min turning radius: %.2f m\n', min_turn_radius);
    
    if min_turn_radius < 0.5
        fprintf('  ⚠ WARNING: Very tight minimum radius (%.2fm)\n', min_turn_radius);
    end
    
    % Maximum curvature
    max_curvature = params.omega_max / params.v_max;
    fprintf('  ✓ Max curvature: %.3f m⁻¹\n', max_curvature);
    
    % Wheel speed limits
    [v_wheel_max, omega_wheel_max] = params.chassis2wheels(params.v_max, params.omega_max);
    fprintf('  ✓ Max wheel speed (straight): %.2f rad/s\n', omega_wheel_max(1));
    
    % Check for wheel slip at max cornering
    wheel_linear_speed = abs(v_wheel_max(1)) * params.wheel_radius;
    if wheel_linear_speed > params.v_max * 1.5
        fprintf('  ⚠ WARNING: Wheel speed very high at max turn\n');
    end
    
    printTestResult('Kinematic Consistency', test_passed);
    all_passed = all_passed && test_passed;
    
        %% Test 7: Utility Functions
    fprintf('\nTest 7: Utility Functions\n');
    test_passed = true;
    
    % Test wrapToPi
    % Note: π and -π represent the same angle, so we need tolerance
    test_angles = [0, pi/2, pi, 3*pi/2, 2*pi, -pi, 5*pi, -5*pi];
    expected = [0, pi/2, pi, -pi/2, 0, -pi, pi, pi];  % π can be ±π
    
    for i = 1:length(test_angles)
        result = params.wrapToPi(test_angles(i));
        
        % Special handling for angles that wrap to ±π (same angle)
        if abs(abs(expected(i)) - pi) < 1e-10
            % At ±π boundary, either π or -π is acceptable
            if abs(abs(result) - pi) > 1e-10
                fprintf('  ✗ FAIL: wrapToPi(%.2f) = %.2f, expected ±π\n', ...
                        test_angles(i), result);
                test_passed = false;
            end
        else
            % Normal case: expect specific value
            if abs(result - expected(i)) > 1e-10
                fprintf('  ✗ FAIL: wrapToPi(%.2f) = %.2f, expected %.2f\n', ...
                        test_angles(i), result, expected(i));
                test_passed = false;
            end
        end
    end
    
    if test_passed
        fprintf('  ✓ wrapToPi function working correctly\n');
    end
    
    % Test angularError
    err1 = params.angularError(0.1, -0.1);
    if abs(err1 - 0.2) > 1e-10
        fprintf('  ✗ FAIL: angularError near zero\n');
        test_passed = false;
    end
    
    err2 = params.angularError(pi - 0.1, -pi + 0.1);
    if abs(err2 - (-0.2)) > 1e-10
        fprintf('  ✗ FAIL: angularError across ±π boundary\n');
        test_passed = false;
    end
    
    if test_passed
        fprintf('  ✓ angularError function working correctly\n');
    end
    
    % Test wheel conversions (round-trip)
    v_test = 1.5;
    omega_test = 0.5;
    [~, omega_wheels] = params.chassis2wheels(v_test, omega_test);
    [v_back, omega_back] = params.wheels2chassis(omega_wheels(1), omega_wheels(2));
    
    if abs(v_back - v_test) > 1e-10 || abs(omega_back - omega_test) > 1e-10
        fprintf('  ✗ FAIL: Wheel conversion round-trip error\n');
        test_passed = false;
    else
        fprintf('  ✓ Wheel conversion round-trip accurate\n');
    end
    
    printTestResult('Utility Functions', test_passed);
    all_passed = all_passed && test_passed;
        
    %% Test 8: Constraint Feasibility
    fprintf('\nTest 8: Constraint Feasibility\n');
    test_passed = true;
    
    % Check that we can actually achieve commanded accelerations
    % given the sampling time
    max_achievable_v_change = params.a_max * params.dt;
    max_achievable_omega_change = params.alpha_max * params.dt;
    
    fprintf('  ✓ Max velocity change per step: %.3f m/s\n', ...
            max_achievable_v_change);
    fprintf('  ✓ Max omega change per step: %.3f rad/s\n', ...
            max_achievable_omega_change);
    
    % Warn if acceleration limits are too restrictive
    if max_achievable_v_change < 0.01
        fprintf('  ⚠ WARNING: Very small velocity changes per step\n');
    end
    
    % Check if we can go from zero to max in reasonable time
    steps_to_max_v = params.v_max / max_achievable_v_change;
    time_to_max_v = steps_to_max_v * params.dt;
    
    fprintf('  ✓ Steps to reach v_max from rest: %.0f (%.2f s)\n', ...
            steps_to_max_v, time_to_max_v);
    
    if steps_to_max_v > params.N
        fprintf('  ⚠ WARNING: Cannot reach max velocity within horizon\n');
    end
    
    printTestResult('Constraint Feasibility', test_passed);
    all_passed = all_passed && test_passed;
    
    %% Final Summary
    fprintf('\n========================================\n');
    fprintf('Parameter Validation Summary\n');
    fprintf('========================================\n');
    
    if all_passed
        fprintf('✓ All parameter tests PASSED\n');
        fprintf('Parameters are physically consistent and reasonable.\n');
    else
        fprintf('✗ Some parameter tests FAILED\n');
        fprintf('Please review the errors above and fix parameters.\n');
    end
    
    fprintf('========================================\n\n');
    
    % Return success/failure
    if ~all_passed
        error('Parameter validation failed. Fix errors before continuing.');
    end
end

%% Helper function
function printTestResult(test_name, passed)
    if passed
        fprintf('  → %s: ✓ PASSED\n', test_name);
    else
        fprintf('  → %s: ✗ FAILED\n', test_name);
    end
end