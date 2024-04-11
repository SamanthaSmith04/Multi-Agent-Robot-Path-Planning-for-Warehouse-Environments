clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm");
grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout2.csv";

% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos = [43,2]*grid_size;% - (grid_size)/2;

%% Set up grid
map = get_map_array(csv_file)
% any other pre-planning operations that need to be done

%% Path planning
RRT(start_pos, goal_pos, map, grid_size)