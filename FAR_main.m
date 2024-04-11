clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "FAR_Code");
grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout1.csv";

% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos = [43,2]*grid_size;% - (grid_size)/2;

%% Set up grid
map = get_map_array(csv_file);
% any other pre-planning operations that need to be done

%% Path planning
G = FlowAnnotatedGraph(map, grid_size);
[start,target] = getStartAndTarget(map,grid_size,start_pos,goal_pos);
node_path = AstarDigraph(G,start,target,map,grid_size,start_pos,goal_pos);