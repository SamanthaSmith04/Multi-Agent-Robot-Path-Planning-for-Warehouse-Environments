clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm");
grid_size = 1; %UNITS????

csv_file = "environment_files/GridLayout1.csv";

csv_file2 = "environment_files/GridLayout1b.csv";


% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos = [43,2]*grid_size;% - (grid_size)/2;

%% Set up grid
map_array = get_map_array(csv_file)
map_array2 = get_map_array(csv_file2)

map = robotics.OccupancyGrid(map_array2, 1);
% any other pre-planning operations that need to be done

%% Path planning
plan = RRT(start_pos, goal_pos, map_array, grid_size, false)

motionPlan_RRT(plan, 45, map, map_array2, grid_size)