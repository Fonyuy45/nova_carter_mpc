%% setup_project.m
% Run this script to initialize the Nova Carter MPC project
% This adds all necessary paths and checks dependencies

function setup_project()
    % Get the project root directory (where this script is located)
    project_root = fileparts(mfilename('fullpath'));
    
    % Add all subdirectories to path
    addpath(genpath(project_root));
    
    % Display project info
    fprintf('\n========================================\n');
    fprintf('Nova Carter MPC Project\n');
    fprintf('========================================\n');
    fprintf('Project root: %s\n', project_root);
    fprintf('MATLAB version: %s\n', version);
    fprintf('Date: %s\n', datestr(now));
    fprintf('========================================\n\n');
    
    % Check for required toolboxes
    fprintf('Checking required toolboxes...\n');
    checkToolbox('Optimization Toolbox', 'optim');
    checkToolbox('Symbolic Math Toolbox', 'symbolic');
    
    % Optional but recommended
    checkToolbox('Control System Toolbox', 'control');
    
    fprintf('\nProject setup complete!\n');
    fprintf('Run tests/test_kinematic_model.m to verify installation.\n\n');
end

function checkToolbox(name, ~)
    v = ver;
    if any(strcmp({v.Name}, name))
        fprintf('  ✓ %s installed\n', name);
    else
        fprintf('  ✗ %s NOT installed (optional for Phase 0)\n', name);
    end
end