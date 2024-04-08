clc;
clear;
close all;

%% Initial Values

grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout1.csv";

% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;
goal_pos = [43,2]*grid_size;

%% Set up grid
map = get_map_array(csv_file)
% any other pre-planning operations that need to be done

%% Path planning
RRT(start_pos, goal_pos, map, grid_size)