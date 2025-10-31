%% test_parameters.m
% Test parameter loading and conversions
function test_parameters()
    close all; clc;
    
    fprintf('Testing nova_carter_params...\n\n');
    
    %% Test 1: Load parameters
    fprintf('Test 1: Parameter loading\n');
    
    try
        params = nova_carter_params;
        
        % Test only properties that exist
        fprintf('  Wheel radius: %.4f m\n', params.wheel_radius);
        
        % Check if wheel_base exists, otherwise use track_width or base_width
        if isprop(params, 'wheel_base')
            fprintf('  Wheel base: %.4f m\n', params.wheel_base);
        elseif isprop(params, 'track_width')
            fprintf('  Track width: %.4f m\n', params.track_width);
        elseif isprop(params, 'base_width')
            fprintf('  Base width: %.4f m\n', params.base_width);
        end
        
        if isprop(params, 'v_max')
            fprintf('  Max velocity: %.2f m/s\n', params.v_max);
        end
        
        if isprop(params, 'dt')
            fprintf('  Time step: %.4f s\n', params.dt);
        end
        
        fprintf('  ✓ Parameters loaded successfully\n\n');
    catch ME
        fprintf('  ✗ Failed to load parameters\n');
        fprintf('  Error: %s\n\n', ME.message);
        rethrow(ME);
    end
    
    %% Test 2: Chassis to wheel conversion
    fprintf('Test 2: Chassis to wheel velocity conversion\n');
    v_chassis = 1.0;  % 1 m/s forward
    omega_chassis = 0.5;  % 0.5 rad/s rotation
    
    try
        [v_wheels, omega_wheels] = params.chassis2wheels(v_chassis, omega_chassis);
        
        fprintf('  Input: v=%.2f m/s, omega=%.2f rad/s\n', v_chassis, omega_chassis);
        fprintf('  Wheel velocities: v_R=%.3f m/s, v_L=%.3f m/s\n', v_wheels(1), v_wheels(2));
        fprintf('  Wheel angular velocities: omega_R=%.3f rad/s, omega_L=%.3f rad/s\n', ...
                omega_wheels(1), omega_wheels(2));
        fprintf('  ✓ Conversion successful\n\n');
    catch ME
        fprintf('  ✗ Chassis to wheel conversion failed\n');
        fprintf('  Error: %s\n\n', ME.message);
        rethrow(ME);
    end
    
    %% Test 3: Round-trip conversion
    fprintf('Test 3: Round-trip conversion test\n');
    
    try
        [v_back, omega_back] = params.wheels2chassis(omega_wheels(1), omega_wheels(2));
        
        error_v = abs(v_back - v_chassis);
        error_omega = abs(omega_back - omega_chassis);
        
        fprintf('  Reconstructed: v=%.6f m/s, omega=%.6f rad/s\n', v_back, omega_back);
        fprintf('  Errors: v_error=%.2e, omega_error=%.2e\n', error_v, error_omega);
        
        if error_v < 1e-6 && error_omega < 1e-6
            fprintf('  ✓ Round-trip conversion successful\n\n');
        else
            fprintf('  ✗ Round-trip conversion failed - errors too large!\n\n');
            error('Conversion test failed: errors exceed threshold');
        end
    catch ME
        fprintf('  ✗ Round-trip conversion failed\n');
        fprintf('  Error: %s\n\n', ME.message);
        rethrow(ME);
    end
    
    %% Test 4: Angle wrapping
    fprintf('Test 4: Angle wrapping\n');
    angles_test = [0, pi/2, pi, 3*pi/2, 2*pi, -pi, -3*pi/2, 5*pi];
    
    fprintf('  Input angles (rad): ');
    fprintf('%.2f ', angles_test);
    fprintf('\n');
    
    try
        % Try different ways to call wrapToPi
        angles_wrapped = zeros(size(angles_test));
        
        for i = 1:length(angles_test)
            if ismethod(params, 'wrapToPi')
                angles_wrapped(i) = params.wrapToPi(angles_test(i));
            else
                % Try as static method
                angles_wrapped(i) = nova_carter_params.wrapToPi(angles_test(i));
            end
        end
        
        fprintf('  Wrapped angles (rad): ');
        fprintf('%.2f ', angles_wrapped);
        fprintf('\n');
        
        % Check all wrapped angles are in [-pi, pi]
        if all(angles_wrapped >= -pi & angles_wrapped <= pi)
            fprintf('  ✓ All angles properly wrapped to [-π, π]\n\n');
        else
            fprintf('  ✗ Angle wrapping failed - angles outside range!\n\n');
            error('Angle wrapping test failed');
        end
    catch ME
        fprintf('  ✗ Angle wrapping test failed\n');
        fprintf('  Error: %s\n\n', ME.message);
        rethrow(ME);
    end
    
    fprintf('All parameter tests passed!\n');
end