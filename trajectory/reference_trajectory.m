%% reference_trajectory.m
% Generate and interpolate reference trajectories

classdef reference_trajectory
    properties
        waypoints   % [x, y, theta, t] array
        type        % 'waypoints', 'circle', 'figure8', 'line'
    end
    
    methods
        function obj = reference_trajectory(type, varargin)
            obj.type = type;
            
            switch type
                case 'circle'
                    radius = varargin{1};
                    center = varargin{2};
                    speed = varargin{3};
                    obj = obj.generateCircle(radius, center, speed);
                    
                case 'figure8'
                    radius = varargin{1};
                    speed = varargin{2};
                    obj = obj.generateFigure8(radius, speed);
                    
                case 'line'
                    start = varargin{1};
                    goal = varargin{2};
                    speed = varargin{3};
                    obj = obj.generateLine(start, goal, speed);
                    
                case 'waypoints'
                    obj.waypoints = varargin{1};
            end
        end
        
        function obj = generateCircle(obj, radius, center, speed)
            % Generate circular trajectory
            dt = nova_carter_params.dt;
            omega = speed / radius;  % angular velocity
            T = 2*pi / omega;        % period
            
            t = 0:dt:T;
            N = length(t);
            
            obj.waypoints = zeros(N, 4);
            for i = 1:N
                angle = omega * t(i);
                obj.waypoints(i,:) = [
                    center(1) + radius * cos(angle),
                    center(2) + radius * sin(angle),
                    angle + pi/2,  % tangent direction
                    t(i)
                ];
            end
        end
        
        function obj = generateFigure8(obj, radius, speed)
            % Generate figure-8 trajectory (lemniscate)
            dt = nova_carter_params.dt;
            T = 20;  % duration (s)
            t = 0:dt:T;
            N = length(t);
            
            obj.waypoints = zeros(N, 4);
            for i = 1:N
                tau = 2*pi * t(i) / T;
                scale = radius * 2;
                
                % Lemniscate of Gerono
                x = scale * cos(tau);
                y = scale * sin(tau) * cos(tau);
                
                % Numerical derivative for heading
                if i < N
                    dx = scale * (-sin(tau)) * 2*pi/T;
                    dy = scale * (cos(2*tau)) * 2*pi/T;
                    theta = atan2(dy, dx);
                else
                    theta = obj.waypoints(i-1, 3);
                end
                
                obj.waypoints(i,:) = [x, y, theta, t(i)];
            end
        end
        
        function obj = generateLine(obj, start, goal, speed)
            % Generate straight line trajectory
            dt = nova_carter_params.dt;
            dist = norm(goal - start);
            T = dist / speed;
            t = 0:dt:T;
            N = length(t);
            
            theta = atan2(goal(2) - start(2), goal(1) - start(1));
            
            obj.waypoints = zeros(N, 4);
            for i = 1:N
                alpha = t(i) / T;  % interpolation parameter
                obj.waypoints(i,:) = [
                    start(1) + alpha * (goal(1) - start(1)),
                    start(2) + alpha * (goal(2) - start(2)),
                    theta,
                    t(i)
                ];
            end
        end
        
        function [x_ref, y_ref, theta_ref] = getReference(obj, t)
            % Interpolate reference at time t
            
            if t <= obj.waypoints(1, 4)
                % Before trajectory starts
                x_ref = obj.waypoints(1, 1);
                y_ref = obj.waypoints(1, 2);
                theta_ref = obj.waypoints(1, 3);
            elseif t >= obj.waypoints(end, 4)
                % After trajectory ends
                x_ref = obj.waypoints(end, 1);
                y_ref = obj.waypoints(end, 2);
                theta_ref = obj.waypoints(end, 3);
            else
                % Interpolate
                x_ref = interp1(obj.waypoints(:,4), obj.waypoints(:,1), t);
                y_ref = interp1(obj.waypoints(:,4), obj.waypoints(:,2), t);
                theta_ref = interp1(obj.waypoints(:,4), obj.waypoints(:,3), t);
                theta_ref = nova_carter_params.wrapToPi(theta_ref);
            end
        end
        
        function plotTrajectory(obj, color)
            % Plot the reference trajectory
            if nargin < 2, color = 'r--'; end
            plot(obj.waypoints(:,1), obj.waypoints(:,2), color, 'LineWidth', 1.5);
            
            % Plot start and goal
            plot(obj.waypoints(1,1), obj.waypoints(1,2), 'go', ...
                 'MarkerSize', 10, 'LineWidth', 2);
            plot(obj.waypoints(end,1), obj.waypoints(end,2), 'rx', ...
                 'MarkerSize', 10, 'LineWidth', 2);
        end
    end
end