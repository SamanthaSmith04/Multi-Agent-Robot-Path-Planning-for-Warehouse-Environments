clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "FAR_Code");
grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout2.csv";

% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos = [25,2]*grid_size;% - (grid_size)/2;


start_pos2 = [10,2]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos2 = [1,25]*grid_size;% - (grid_size)/2;

start_pos3 = [35,20]*grid_size;% - (grid_size)/2; (To center within grid Point)
goal_pos3 = [2,12]*grid_size;% - (grid_size)/2;


%% Set up grid
map = get_map_array(csv_file);
% any other pre-planning operations that need to be done

%% Path planning
fig_num = 1;
G = FlowAnnotatedGraph(map, grid_size);
[start,target] = getStartAndTarget(map,grid_size,start_pos,goal_pos);
node_path = AstarDigraph(G,start,target,map,grid_size,start_pos,goal_pos,fig_num);

[start2,target2] = getStartAndTarget(map,grid_size,start_pos2,goal_pos2);
node_path2 = AstarDigraph(G,start2,target2);

[start3,target3] = getStartAndTarget(map,grid_size,start_pos3,goal_pos3);
node_path3 = AstarDigraph(G,start3,target3);

%Add Road With points to each bot
x_coo = G.Nodes.X(node_path);
y_coo = G.Nodes.Y(node_path);
road1 = [start_pos; x_coo,y_coo; goal_pos];
%Add Road With points to each bot
x_coo = G.Nodes.X(node_path2);
y_coo = G.Nodes.Y(node_path2);
road2 = [start_pos2; x_coo,y_coo; goal_pos2];
%Add Road With points to each bot
x_coo = G.Nodes.X(node_path3);
y_coo = G.Nodes.Y(node_path3);
road3 = [start_pos3; x_coo,y_coo; goal_pos3];

bot1.node_path = node_path;
bot2.node_path = node_path2;
bot3.node_path = node_path3;

bot1.start = start_pos;
bot2.start = start_pos2;
bot3.start = start_pos3;

bot1.goal = goal_pos;
bot2.goal = goal_pos2;
bot3.goal = goal_pos3;

bot1.road = road1;
bot2.road = road2;
bot3.road = road3;

robot_array = [bot1, bot2, bot3];
k = 40; %reserve upto 5 steps (for now)
timeStep = 0.1;
[simulated_robots] = FARMultiRobotController(G,robot_array,k,timeStep);

figureNumber = 2;
%[done] = vizualizeFARmotion3(simulated_robots,timeStep,map,grid_size,figureNumber);
[dist, avgDist, stdDist] = distanceBetweenRobot(simulated_robots);




