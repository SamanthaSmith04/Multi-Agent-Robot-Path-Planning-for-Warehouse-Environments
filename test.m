get_map_array("environment_files/GridLayout2.csv");
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
load exampleMap

grid_size = 1; %UNITS????

csv_file = "environment_files/GridLayout1.csv";

obstacle_csv = "environment_files/GridLayout1b.csv";


% temp positions for testing, can be random later if we want
start_pos = [1,1]*grid_size;
goal_pos = [43,2]*grid_size;

%% Set up grid
map_array = get_map_array(csv_file);
map = robotics.OccupancyGrid(get_map_array(obstacle_csv), 1);

% any other pre-planning operations that need to be done

%% Path planning
waypoints = RRT(start_pos, goal_pos, map_array, grid_size)


% Define Vehicle
R = 0.1;                        % Wheel radius [m]
L = 0.5;                        % Wheelbase [m]
dd = DifferentialDrive(R,L);
% Sample time and time array
sampleTime = 0.1;              % Sample time [s]
% Initial conditions
tVec = 0:sampleTime:size(waypoints,1);        % Time array
initPose = [1;1;0];            % Initial pose (x y theta)
pose = zeros(3,numel(tVec));   % Pose matrix
pose(:,1) = initPose;
% Load map
% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,51);
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
% Vector Field Histogram (VFH) for obstacle avoidance
% vfh = controllerVFH;
% vfh.DistanceLimits = [0.05 3];
% vfh.NumAngularSectors = 36;
% vfh.HistogramThresholds = [5 10];
% vfh.RobotRadius = L;
% vfh.SafetyDistance = L;
% vfh.MinTurningRadius = 0.25;
%% Simulation loop
current_waypoint_idx = 1;
r = rateControl(1/sampleTime);
%for idx = 2:numel(tVec) 
idx = 2
 while (current_waypoint_idx < size(waypoints,1))   
    % Get the sensor readings
    curPose = pose(:,idx-1);
    ranges = lidar(curPose);
    % Check if the robot is close to the current waypoint
    if norm(curPose(1:2) - waypoints(current_waypoint_idx, :).') < 0.1
        current_waypoint_idx = current_waypoint_idx + 1; % Move to the next waypoint
        if current_waypoint_idx > size(waypoints, 1)
            disp("AAAAAAAAAAAAA")
            break; % Reached the last waypoint, end simulation
           
        end
    end
% 
%     % Run the path following and obstacle avoidance algorithms
    [vRef,wRef,lookAheadPt] = controller(curPose);
%     targetDir = atan2(lookAheadPt(2)-curPose(2),lookAheadPt(1)-curPose(1)) - curPose(3);
%     steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
%     if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
%         wRef = 0.5*steerDir;
%     end
%     
%     % Control the robot
%     velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
%     vel = bodyToWorld(velB,curPose);  % Convert from body to world
    velB = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,curPose);  % Convert from body to world
%     
    [waypoints, changed] = dynamic_RRT_replanning(waypoints, current_waypoint_idx, lidar, curPose, controller.MaxAngularVelocity, vel, map,grid_size);
    % Perform forward discrete integration step
    if changed == true
        pose(:,end+1) = zeros(3,1);
        tVec(end+1) = tVec(end) + sampleTime;
        controller.Waypoints = waypoints;
    end
    pose(:,idx) = curPose + vel*sampleTime;
    % Update visualization
    viz(pose(:,idx),waypoints,ranges)
    waitfor(r);
    idx = idx +1;
end