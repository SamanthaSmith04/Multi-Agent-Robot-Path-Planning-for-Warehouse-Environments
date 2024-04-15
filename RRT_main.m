clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "motion_planning");
grid_size = 1; %UNITS????

csv_file = "environment_files/GridLayout2.csv";

csv_file2 = "environment_files/GridLayout2.csv";
  
num_robots = 2

% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos = [43,2]*grid_size;% - (grid_size)/2;


start_pos2 = [1,13]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos2 = [43,25]*grid_size;% - (grid_size)/2;

start_pos3 = [2,24]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos3 = [40,15]*grid_size;% - (grid_size)/2;;

%% Set up grid
map_array = get_map_array(csv_file)
map_array2 = get_map_array(csv_file2)

map = robotics.OccupancyGrid(map_array2, 1);
% any other pre-planning operations that need to be done

%% Path planning
bot1.start = start_pos;
bot2.start = start_pos2;
bot3.start = start_pos3;

bot1.goal = goal_pos;
bot2.goal = goal_pos2;
bot3.goal = goal_pos3;

road1 = RRT(start_pos, goal_pos, map_array, grid_size, false)
road2 = RRT(start_pos2, goal_pos2, map_array, grid_size, false)
road3 = RRT(start_pos3, goal_pos3, map_array, grid_size, false)

bot1.road = road1;
bot2.road = road2;
bot3.road = road3;

bot1.thetaInit = 45;
bot2.thetaInit = 45;
bot3.thetaInit = 45;

robots = [bot1, bot2]
disp("ready")
[phi, time, position_time_matrix] = motionPlan_RRT_multiple(robots, map, map_array, grid_size)
%[phi, time, position_time_matrix] = motionPlan_RRT(plan1, 45, map, map_array2, grid_size)