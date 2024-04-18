clc;
clear;
close all;

%% EXAMPLE: Differential Drive Path Following
% In this example, a differential drive robot navigates a set of waypoints 
% using the Pure Pursuit algorithm while avoiding obstacles using the
% Vector Field Histogram (VFH) algorithm.
% 
% Copyright 2019 The MathWorks, Inc.
%% Simulation setup

% Create waypoints
%% Initial Values
%load exampleMap

grid_size = 1; %UNITS????

csv_file = "/../environment_files/GridLayout1.csv";

obstacle_csv = "/../environment_files/GridLayout1b.csv";


% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;
goal_pos = [43,2]*grid_size;

%% Set up grid
map_array = get_map_array(csv_file);
map = robotics.OccupancyGrid(get_map_array(obstacle_csv), 1);

% any other pre-planning operations that need to be done

%% Path planning
waypoints = RRT(start_pos, goal_pos, map_array, grid_size, false)


% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);
% Sample time and time array
sampleTime = 0.08;              % Sample time [s]
% Initial conditions
tVec = 0:sampleTime:size(waypoints,1);        % Time array
initPose = [1;1;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,90);
lidar.maxRange = 5;
% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);
%% Path planning and following



% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.5;
controller.DesiredLinearVelocity = 0.75;
controller.MaxAngularVelocity = 1.5;

%% Simulation loop
current_waypoint_idx = 2;
r = rateControl(1/sampleTime);
%for idx = 2:numel(tVec) 
idx = 2
curPose = pose(:,idx-1);
numel(tVec)
waypoints_added = 0;
while ~has_reached_goal(curPose, goal_pos)
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
    % min(ranges)

    % Run the path following algorithm
    [vRef,wRef,lookAheadPt] = controller(curPose);

    % Control the robot
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
    
    % Perform dynamic replanning
    [waypoints, changed, current_index_update] = dynamic_RRT_replanning(waypoints, current_waypoint_idx, lidar, curPose, controller.MaxAngularVelocity, vel, map_array, grid_size, L);
    
    % Update controller waypoints and internal state


    if changed

        pose(:, end + 1) = zeros(3,1);
        tVec(end+1) = tVec(end) + sampleTime;
        waypoints_added = waypoints_added +1;
        current_waypoint_idx = current_waypoint_idx + current_index_update;
        release(controller);
        controller.Waypoints = waypoints;
    end


    % Check if the waypoint has been reached
    if current_waypoint_idx < size(waypoints, 1) && distance(curPose(1), curPose(2), waypoints(current_waypoint_idx, 1), waypoints(current_waypoint_idx, 2)) < 0.75
        current_waypoint_idx = current_waypoint_idx + 1 % Move to the next waypoint
    end



    % Perform forward discrete integration step
    pose(:,idx) = curPose + vel*sampleTime;
    
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    idx = idx + 1;
    waitfor(r);
end