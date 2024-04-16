clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "motion_planning");
grid_size = 1; %UNITS????

csv_file = "environment_files/GridLayout2.csv";

csv_file2 = "environment_files/GridLayout2.csv";

robot_positions_csv = ""

robot_positions = get_position_and_goals(robot_positions_csv);

%% Set up grid
map_array = get_map_array(csv_file)
map_array2 = get_map_array(csv_file2)

map = robotics.OccupancyGrid(map_array2, 1);
% any other pre-planning operations that need to be done
for num_robots=1: size(robot_positions,1)
    robot.start = robot_positions(num_robots, 1:2);
    robot.goal = robot_positions(num_robots, 3:4);
    robot.road = RRT(robot.start, robot.goal, map_array, grid_size, false);
    robot.original_road = robot.road;
    robot.thetaInit = 45;

    robots(i) = robot;
end

disp("ready")
[phi, time, robots] = motionPlan_RRT_multiple(robots, map, map_array2, grid_size);
%[phi, time, position_time_matrix] = motionPlan_RRT(plan1, 45, map, map_array2, grid_size)