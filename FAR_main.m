clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "FAR_Code");
grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout2.csv";


robot_positions_csv = "Experiment_CSV/exp1_both.csv";

robot_positions = get_position_and_goals(robot_positions_csv);

%% Set up grid
map = get_map_array(csv_file);
% any other pre-planning operations that need to be done

%% Path planning
fig_num = 1;
G = FlowAnnotatedGraph(map, grid_size);
for num_robots=1: size(robot_positions,1)
    robot.start = robot_positions(num_robots, 1:2)*grid_size;
    robot.goal = robot_positions(num_robots, 3:4)*grid_size;
    robot.road = G
    robot.original_road = robot.road;
    robot.thetaInit = 45;

    robots(num_robots) = robot;
end

if size(robots) >= 1
    [start,target] = getStartAndTarget(map,grid_size,robots(1).start,robots(1).goal);
    node_path = AstarDigraph(G,start,target,map,grid_size,robots(1).start,robots(1).goal,fig_num);
    %Add Road With points to each bot
    x_coo = G.Nodes.X(node_path);
    y_coo = G.Nodes.Y(node_path);
    road1 = [robots(1).start; x_coo,y_coo; robots(1).goal];
    robots(1).node_path = node_path;
end

if size(robots) >= 2
    [start2,target2] = getStartAndTarget(map,grid_size,robots(2).start,robots(2).goal);
    node_path2 = AstarDigraph(G,start2,target2);
    %Add Road With points to each bot
    x_coo = G.Nodes.X(node_path2);
    y_coo = G.Nodes.Y(node_path2);
    road2 = [start_pos2; x_coo,y_coo; robots(2).goal];
    robots(2).node_path = node_path2;
end

if size(robots) >= 3
    [start3,target3] = getStartAndTarget(map,grid_size,robots(3).start,robots(3).goal);
    node_path3 = AstarDigraph(G,start3,target3);
    %Add Road With points to each bot
    x_coo = G.Nodes.X(node_path3);
    y_coo = G.Nodes.Y(node_path3);
    road3 = [start_pos3; x_coo,y_coo; robots(3).goal];
    robots(3).node_path = node_path3;
end


k = 40; %reserve upto 5 steps (for now)
timeStep = 0.1;
[simulated_robots] = FARMultiRobotController(G,robots,k,timeStep);

figureNumber = 2;
%[done] = vizualizeFARmotion3(simulated_robots,timeStep,map,grid_size,figureNumber);
[dist, avgDist, stdDist] = distanceBetweenRobot(simulated_robots);




