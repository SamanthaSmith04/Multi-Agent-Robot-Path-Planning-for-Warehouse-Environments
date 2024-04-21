clc;
clear;
close all;

%% Initial Values
addpath("environment_files", "initialization", "RRT_algorithm", "FAR_Code", "data_plotting");
grid_size = 2; %UNITS????

csv_file = "environment_files/GridLayout1.csv";
robot_positions_csv = "Experiment_CSV/exp1_both.csv";
outputStructFileName = "Experiment_Results/FAR_exp1map1.xml";

robot_positions = get_position_and_goals(robot_positions_csv)*grid_size;

%% Set up grid
map = get_map_array(csv_file);
% any other pre-planning operations that need to be done

%% Path planning
fig_num = 1;
G = FlowAnnotatedGraph(map, grid_size);
for num_robots=1: size(robot_positions,1)
    robot.start = robot_positions(num_robots, 1:2);
    robot.goal = robot_positions(num_robots, 3:4);
    robot.road = [];
    robot.original_road = robot.road;
    robot.thetaInit = 45;

    robots(num_robots) = robot;
end

if length(robots) >= 1
    [start,target] = getStartAndTarget(map,grid_size,robots(1).start,robots(1).goal);
    
    %node_path = AstarDigraph(G,start,target,map,grid_size,robots(1).start,robots(1).goal,fig_num);
    node_path = AstarDigraph(G,start,target);
    %Add Road With points to each bot
    x_coo = G.Nodes.X(node_path);
    y_coo = G.Nodes.Y(node_path);
    road1 = [robots(1).start; x_coo,y_coo; robots(1).goal];
    robots(1).node_path = node_path;
    robots(1).road = road1;
end

if length(robots) >= 2
    [start2,target2] = getStartAndTarget(map,grid_size,robots(2).start,robots(2).goal);
    %node_path2 = AstarDigraph(G,start2,target2,map,grid_size,robots(2).start,robots(2).goal,fig_num);
    node_path2 = AstarDigraph(G,start2,target2);
    %Add Road With points to each bot
    x_coo = G.Nodes.X(node_path2);
    y_coo = G.Nodes.Y(node_path2);
    road2 = [robots(2).start; x_coo,y_coo; robots(2).goal];
    robots(2).node_path = node_path2;
    robots(2).road = road2;
end

if length(robots) >= 3
    [start3,target3] = getStartAndTarget(map,grid_size,robots(3).start,robots(3).goal);
    node_path3 = AstarDigraph(G,start3,target3);
    %Add Road With points to each bot
    x_coo = G.Nodes.X(node_path3);
    y_coo = G.Nodes.Y(node_path3);
    disp(robots)
    road3 = [robots(3).start; x_coo,y_coo; robots(3).goal];
    
    robots(3).node_path = node_path3;
    robots(3).road = road3;
end


k = 5; %reserve upto 5 steps (for now)
timeStep = 0.1;
[simulated_robots] = FARMultiRobotController(G,robots,k,timeStep);

figureNumber = 2;
%[done] = vizualizeFARmotion3(simulated_robots,timeStep,map,grid_size,figureNumber);
[dist, avgDist, stdDist] = distanceBetweenRobot(simulated_robots);
FAR_plot_robot_paths(simulated_robots, length(simulated_robots), map, grid_size)

%%Code Below To Save Results (Path name specified at top)
simulated_robots = pathLength(simulated_robots);
write_exp_struct(simulated_robots,outputStructFileName)

tempStruct = read_exp_struct(outputStructFileName);