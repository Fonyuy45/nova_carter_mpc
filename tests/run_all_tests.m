%% run_all_tests.m
% Master test script to run all validation tests
function run_all_tests()
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Running Nova Carter Test Suite\n');
    fprintf('========================================\n\n');
    
    % Define test functions
    test_functions = {
        @test_parameters
        @test_kinematic_model
        @test_open_loop_wheel_commands
        
        % Add more test functions here as you create them
    };
    
    test_names = {
        'test_parameters'
        'test_kinematic_model'
        'test_open_loop_wheel_commands'
    };
    
    % Initialize results structure
    results = struct();
    
    % Run each test
    for i = 1:length(test_functions)
        test_name = test_names{i};
        fprintf('Running %s...\n', test_name);
        fprintf('----------------------------------------\n');
        
        try
            % Run the test function
            test_functions{i}();
            results.(test_name) = 'PASSED';
            fprintf('----------------------------------------\n');
            fprintf('✓ %s PASSED\n\n', test_name);
        catch ME
            results.(test_name) = 'FAILED';
            fprintf('----------------------------------------\n');
            fprintf('✗ %s FAILED\n', test_name);
            fprintf('  Error: %s\n', ME.message);
            if ~isempty(ME.stack)
                fprintf('  Location: %s (line %d)\n\n', ...
                        ME.stack(1).name, ME.stack(1).line);
            else
                fprintf('  (no stack trace available)\n\n');
            end
        end
    end
    
    % Summary
    fprintf('========================================\n');
    fprintf('Test Suite Complete\n');
    fprintf('========================================\n');
    
    % Count results
    test_result_names = fieldnames(results);
    passed = 0;
    failed = 0;
    
    for i = 1:length(test_result_names)
        if strcmp(results.(test_result_names{i}), 'PASSED')
            passed = passed + 1;
        else
            failed = failed + 1;
        end
    end
    
    total = length(test_result_names);
    fprintf('Results: %d/%d tests passed', passed, total);
    if failed > 0
        fprintf(', %d failed', failed);
    end
    fprintf('\n');
    
    if passed == total
        fprintf('✓ All tests passed!\n');
    else
        fprintf('✗ Some tests failed. Review output above.\n');
    end
    fprintf('========================================\n\n');
    
    % Display detailed results
    fprintf('Detailed Results:\n');
    for i = 1:length(test_result_names)
        status = results.(test_result_names{i});
        if strcmp(status, 'PASSED')
            fprintf('  ✓ %s: PASSED\n', test_result_names{i});
        else
            fprintf('  ✗ %s: FAILED\n', test_result_names{i});
        end
    end
    fprintf('\n');
end