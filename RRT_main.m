clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "motion_planning");
grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout1.csv";

robot_positions_csv = "Experiment_CSV/exp4_both.csv";

robot_positions = get_position_and_goals(robot_positions_csv);

%% Set up grid
map_array = get_map_array(csv_file);

% any other pre-planning operations that need to be done
for num_robots=1: size(robot_positions,1)
    robot.start = robot_positions(num_robots, 1:2)*grid_size;
    robot.goal = robot_positions(num_robots, 3:4)*grid_size;
    robot.road = RRT(robot.start, robot.goal, map_array, grid_size, false);
    robot.original_road = robot.road;
    robot.thetaInit = 45;

    robots(num_robots) = robot;
end

scaled_matrix = zeros(size(map_array, 1) * grid_size, size(map_array, 2) * grid_size);
for i = 1:size(map_array, 1)
    for j = 1:size(map_array, 2)
        scaled_matrix((i-1)*grid_size+1:i*grid_size, (j-1)*grid_size+1:j*grid_size) = map_array(i, j);
    end
end

map = robotics.OccupancyGrid(scaled_matrix, 1);

disp("ready")
[phi, time, robots] = motionPlan_RRT_multiple(robots, map, map_array, grid_size, true, 14);