%% robot_state.m
% State representation for the robot

classdef robot_state
    properties
        x       % x position (m)
        y       % y position (m)
        theta   % heading angle (rad)
        t       % time (s)
    end
    
    methods
        function obj = robot_state(x, y, theta, t)
            if nargin < 4, t = 0; end
            obj.x = x;
            obj.y = y;
            obj.theta = nova_carter_params.wrapToPi(theta);
            obj.t = t;
        end
        
        function vec = toVector(obj)
            % Convert to column vector [x; y; theta]
            vec = [obj.x; obj.y; obj.theta];
        end
        
        function obj = fromVector(obj, vec, t)
            % Create from vector
            if nargin < 3, t = 0; end
            obj.x = vec(1);
            obj.y = vec(2);
            obj.theta = nova_carter_params.wrapToPi(vec(3));
            obj.t = t;
        end
        
        function plotRobot(obj, color)
            % Plot robot as oriented triangle
            if nargin < 2, color = 'b'; end
            
            % Robot triangle (pointing forward)
            L = 0.3;  % triangle size
            robot_shape = [L, 0; -L/2, L/2; -L/2, -L/2; L, 0]';
            
            % Rotation matrix
            R = [cos(obj.theta), -sin(obj.theta);
                 sin(obj.theta),  cos(obj.theta)];
            
            % Transform and plot
            robot_global = R * robot_shape + [obj.x; obj.y];
            plot(robot_global(1,:), robot_global(2,:), color, 'LineWidth', 2);
            
            % Add heading indicator
            heading_end = [obj.x; obj.y] + 0.25*[cos(obj.theta); sin(obj.theta)];
            plot([obj.x, heading_end(1)], [obj.y, heading_end(2)], ...
                 color, 'LineWidth', 2);
        end
    end
end