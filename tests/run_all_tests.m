%% run_all_tests.m
% Master test script to run all validation tests
function run_all_tests()
    fprintf('\n');
    fprintf('========================================\n');
    fprintf('Running Nova Carter Test Suite\n');
    fprintf('========================================\n\n');
    
    % Define test files
    test_files = {
        'test_parameters'
        'test_kinematic_model'
        % Add more test files here as you create them
    };
    
    % Initialize results structure
    results = struct();
    
    % Run each test
    for i = 1:length(test_files)
        test_name = test_files{i};
        fprintf('Running %s...\n', test_name);
        
        try
            % Run the test
            run(test_name);
            results.(test_name) = 'PASSED';
            fprintf('  ✓ PASSED\n\n');
        catch ME
            % Fix: Use test_files{i} instead of test_name (which is cleared)
            results.(test_files{i}) = 'FAILED';
            fprintf('  ✗ FAILED: %s\n', ME.message);
            if ~isempty(ME.stack)
                fprintf('     at: %s (line %d)\n\n', ...
                        ME.stack(1).name, ME.stack(1).line);
            else
                fprintf('     (no stack trace available)\n\n');
            end
        end
    end
    
    % Summary
    fprintf('========================================\n');
    fprintf('Test Suite Complete\n');
    fprintf('========================================\n');
    
    % Count results
    test_names = fieldnames(results);
    passed = 0;
    failed = 0;
    
    for i = 1:length(test_names)
        if strcmp(results.(test_names{i}), 'PASSED')
            passed = passed + 1;
        else
            failed = failed + 1;
        end
    end
    
    total = length(test_names);
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
    for i = 1:length(test_names)
        status = results.(test_names{i});
        if strcmp(status, 'PASSED')
            fprintf('  ✓ %s: PASSED\n', test_names{i});
        else
            fprintf('  ✗ %s: FAILED\n', test_names{i});
        end
    end
    fprintf('\n');
end