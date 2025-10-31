%% plotting_utils.m
% Reusable plotting functions for the project

classdef plotting_utils
    methods (Static)
        function fig = setup_trajectory_plot(title_str)
            % Create a standard trajectory plot with nice formatting
            fig = figure('Name', title_str, 'Position', [100, 100, 800, 600]);
            hold on; grid on; axis equal;
            xlabel('X Position (m)', 'FontSize', 12);
            ylabel('Y Position (m)', 'FontSize', 12);
            title(title_str, 'FontSize', 14, 'FontWeight', 'bold');
            set(gca, 'FontSize', 11);
        end
        
        function plot_robot_footprint(x, y, theta, color)
            % Plot robot as a rectangle with heading indicator
            if nargin < 4, color = 'b'; end
            
            params = nova_carter_params;
            L = params.length;
            W = params.width;
            
            % Rectangle corners (local frame)
            corners = [
                L/2,  W/2;
                L/2, -W/2;
               -L/2, -W/2;
               -L/2,  W/2;
                L/2,  W/2
            ]';
            
            % Rotation matrix
            R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            
            % Transform to global frame
            corners_global = R * corners + [x; y];
            
            plot(corners_global(1,:), corners_global(2,:), ...
                 'Color', color, 'LineWidth', 2);
            
            % Heading arrow
            arrow_length = L * 0.6;
            arrow_end = [x; y] + arrow_length * [cos(theta); sin(theta)];
            quiver(x, y, arrow_end(1)-x, arrow_end(2)-y, 0, ...
                   'Color', color, 'LineWidth', 2, 'MaxHeadSize', 0.5);
        end
        
        function save_figure(fig, filename, formats)
            % Save figure in multiple formats
            % formats: cell array like {'png', 'pdf', 'fig'}
            if nargin < 3, formats = {'png'}; end
            
            for i = 1:length(formats)
                fmt = formats{i};
                saveas(fig, filename, fmt);
                fprintf('Saved: %s.%s\n', filename, fmt);
            end
        end
    end
end