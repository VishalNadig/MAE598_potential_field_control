clear;
clc;
% rng(2);

% Constants
ft = 5*60;
dt = 0.1;
DIST_THRESHOLD = 0.1;
NUM_OBSTACLES = 5;
NUM_CHARGERS = 1; % Change this to the desired number of charging stations
NUM_ROBOTS = 5;
INITIAL_BATTERY = 100;
BATTERY_DECREASE_RATE = 20*(dt / 720) * 100;

% Grid
[X, Y] = meshgrid(0:0.1:25, 0:0.1:25);

% Obstacle positions
obstacle_positions = [6 14; 12 3; 12 18; 8 8; 14 5];

% Goal and charger positions
% goal_positions = generate_random_positions(1, X, Y);
% charger_positions = generate_random_positions(NUM_CHARGERS, X, Y);
% home_positions = generate_random_positions(1, X, Y);

goal_positions = [8 13];
charger_positions = [4 18];
home_positions = [4 15];



% Colors for plotting
colors = {'red', 'blue', 'black', 'cyan', 'magenta'};

% Potential fields
obs_pot = calculate_obstacle_potential(X, Y, obstacle_positions);
goal_pot = calculate_goal_potential(X, Y, goal_positions);
home_pot = calculate_goal_potential(X, Y, home_positions);
charger_pot = calculate_charger_potential(X, Y, charger_positions);
bound_pot = calculate_boundary_potential(X, Y);

figure(1)
surf(X, Y, goal_pot, 'EdgeColor', 'none')
title('Goal Potential')

figure(2)
surf(X, Y, obs_pot, 'EdgeColor', 'none')
title('Obstacle Potential')

figure(3)
surf(X, Y, home_pot, 'EdgeColor', 'none')
title('Home Potential')

figure(4)
surf(X, Y, charger_pot, 'EdgeColor', 'none')
title('Charger Potential')

% Plotting the potential field
figure(5);clf;
Z = goal_pot+obs_pot+charger_pot+ bound_pot;
surf(X, Y, Z, 'EdgeColor', 'none');
title('SUM')


% Initial robot positions and parameters
robot_positions = [4 4; 18 6; 20 14; 2 15; 18 18];
robot_battery_status = ones(NUM_ROBOTS, 1) * INITIAL_BATTERY;

battery_record = robot_battery_status;
robot_direction = zeros(NUM_ROBOTS, 1);

step_size = 0.2;

str = '#FFC0CB';
pink_color = sscanf(str(2:end),'%2x%2x%2x',[1 3])/255;

% Plotting circles for visualization
figure(6); clf;
hold on; grid on; grid minor;
scatter(goal_positions(1), goal_positions(2), 500, 'green', 'filled');
scatter(obstacle_positions(:, 1), obstacle_positions(:, 2), 200, 'r', 'filled');
scatter(charger_positions(:, 1), charger_positions(:, 2), 500, 'y', 'filled');
scatter(home_positions(1), home_positions(2), 500, pink_color, 'filled');
title('Robot Paths and Map')
hold on;


% Initialize a matrix to store the battery status of all robots over time
% battery_status_matrix = zeros(NUM_ROBOTS, ft/dt);

% Robot simulation
for sim_t = 1:dt:ft
    for i = 1:NUM_ROBOTS
        x = robot_positions(i, 1);
        y = robot_positions(i, 2);
        path = [x, y];

        robot_pot = 0.2*(calculate_robot_potential(X, Y, robot_positions) - calculate_robot_potential(X, Y, [x y]));

        for t = dt:dt:1
            % Numerical gradient calculation using finite differences
            alpha = 75/robot_battery_status(i);
            
            if robot_direction(i) == 0
                if robot_battery_status(i) > 70
                    Z = goal_pot + obs_pot ;
                else
                    Z = goal_pot + obs_pot + alpha*charger_pot ;
                end
                figure(7)
                surf(X, Y, Z, 'EdgeColor','none');
                title('Towards Goal Pot')
            else
                if robot_battery_status(i) > 70
                    Z = home_pot + obs_pot ;
                else
                    Z = home_pot + obs_pot + alpha*charger_pot;
                end
                figure(8)
                surf(X, Y, Z, 'EdgeColor','none');
                title('Towards Home Pot')
            end

            [grad_x, grad_y] = gradient(Z, step_size, step_size);

            % Index to find the closest coordinate in the grid
            [~, x_index] = min(abs(X(1, :) - x));
            [~, y_index] = min(abs(Y(:, 1) - y));

            % Calculate the components of the gradient
            gx = grad_x(y_index, x_index);
            gy = grad_y(y_index, x_index);
            
            gx =  gx./sqrt((gx).^2 + (gy).^2);
            gy =  gy./sqrt((gx).^2 + (gy).^2);


            x = x - step_size * gx;
            y = y - step_size * gy;

            % Update battery life
            dist_to_chargers = sqrt((x - charger_positions(:, 1)).^2 + (y - charger_positions(:, 2)).^2);
            if any(dist_to_chargers < DIST_THRESHOLD)
                robot_battery_status(i) = INITIAL_BATTERY;
            else
                robot_battery_status(i) = robot_battery_status(i) - BATTERY_DECREASE_RATE;
%                 disp([i, robot_battery_status(i)])
            end
            
%             battery_status_matrix(round(sim_t/dt), i) = robot_battery_status(i);

            % Store the updated position in the path
            path = [path; x, y];

            % Check if the robot has reached the target or home position
            if norm([x, y] - goal_positions) < DIST_THRESHOLD
                robot_direction(i) = 2;
            elseif norm([x, y] - home_positions) < DIST_THRESHOLD
                robot_direction(i) = 0;
            end
        end
       

        figure(6);

        % Plotting the path for each robot in different colors
        plot(path(:, 1), path(:, 2), 'LineWidth', 2, 'Color', colors{i});
        legend('Initial Goal', 'Obstacle', 'Battery Charging Station', 'Home Position', 'Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5')

        drawnow; % Render the plot

        % Update robot position
        robot_positions(i, :) = [x, y];


        
    end
    battery_record = [battery_record robot_battery_status];

    figure(9);
    hold on;
    for i = 1:NUM_ROBOTS
        plot(1-dt:dt:sim_t, battery_record(i , :), 'LineWidth', 2, 'Color', colors{i});
    end
    title('Battery Status of Robots Over Time');
    xlabel('Time');
    ylabel('Battery Status');
    legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5'); % Update legend accordingly
    hold off;
    drawnow;

end

% legend('Target', 'Obstacle', 'Charging Stations', 'Home');

% Function to calculate obstacle potential
function obs_pot = calculate_obstacle_potential(X, Y, obstacle_positions)
    obs_pot = 0;
    A = 5000;
    sigma_sq = 0.4;
    for i = 1:size(obstacle_positions, 1)
        obs_pot = obs_pot + A * exp(-((X - obstacle_positions(i, 1)).^2 + (Y - obstacle_positions(i, 2)).^2) / (2 * sigma_sq));
    end
end

function obs_pot = calculate_robot_potential(X, Y, obstacle_positions)
    obs_pot = 0;
    A = 5000;
    sigma_sq = 0.01;
    for i = 1:size(obstacle_positions, 1)
        obs_pot = obs_pot + A * exp(-((X - obstacle_positions(i, 1)).^2 + (Y - obstacle_positions(i, 2)).^2) / (2 * sigma_sq));
    end
end


% Function to calculate goal potential
function goal_pot = calculate_goal_potential(X, Y, goal_positions)
    goal_pot = 20*((X - goal_positions(1)).^2 + (Y - goal_positions(2)).^2) - 1./(sqrt((X - goal_positions(1)).^2 + (Y - goal_positions(2)).^2)).^2;
end

% Function to calculate goal potential
function home_pot = calculate_home_potential(X, Y, home_positions)
    home_pot = 20*((X - home_positions(1)).^2 + (Y - home_positions(2)).^2) - 1./(sqrt((X - home_positions(1)).^2 + (Y - home_positions(2)).^2)).^2;
end

% Function to calculate charger potential for a specific robot position
function charger_pot = calculate_charger_potential(X, Y, charger_positions)
    charger_pot = 18*((X - charger_positions(1)).^2 + (Y - charger_positions(2)).^2) - 1./(sqrt((X - charger_positions(1)).^2 + (Y - charger_positions(2)).^2)).^2;
end

% Function to generate random positions for charging stations within the grid
function charger_positions = generate_random_positions(num_chargers, X, Y)
    min_x = min(X(:));
    max_x = max(X(:));
    min_y = min(Y(:));
    max_y = max(Y(:));
    
    charger_positions = [rand(1, num_chargers) * (max_x - min_x) + min_x; rand(1, num_chargers) * (max_y - min_y) + min_y]';
    disp(charger_positions)
end


function bound_pot = calculate_boundary_potential(X, Y)
    A = 500;
    sigma_sq = 0.4;
    bound_pot = 0; % Initialize boundary potential

    % Creating obstacles at the boundary
    min_x = min(X(:));
    max_x = max(X(:));
    min_y = min(Y(:));
    max_y = max(Y(:));
    step = 2; % Distance between obstacles

    % Obstacles on top and bottom edges
    for x = min_x:step:max_x
        % Boundary potential calculation
        bound_pot = bound_pot + A*sum(exp(-((X - x).^2 + (Y - min_y).^2) / (2 * sigma_sq)), 'all');
        bound_pot = bound_pot + sum(exp(-((X - x).^2 + (Y - max_y).^2) / (2 * sigma_sq)), 'all');
    end

    % Obstacles on left and right edges (excluding corners to avoid duplicates)
    for y = min_y + step:step:max_y - step
        % Boundary potential calculation
        bound_pot = bound_pot + sum(exp(-((X - min_x).^2 + (Y - y).^2) / (2 * sigma_sq)), 'all');
        bound_pot = bound_pot + sum(exp(-((X - max_x).^2 + (Y - y).^2) / (2 * sigma_sq)), 'all');
    end
end

