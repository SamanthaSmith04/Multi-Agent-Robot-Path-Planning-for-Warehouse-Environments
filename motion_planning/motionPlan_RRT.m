function [phi, time, position_time_matrix] = motionPlan_RRT(road, thetaInit, map, map_array, scale)

%phi is the angular velocity of each of the wheels at each instance of
%sampletime
%time is the overall time vector for the planning
%position_time_matrix is the position and timestamp of each robot pose at
%each sample time
%road is the road of the path planned from the algorithms
%thetaInit is the initial angle of the robot from the x axis
robotInitialLocation = road(1,:);
robotGoal            = road(end,:);

initialOrentation    = thetaInit;

robotCurrentPose     = [robotInitialLocation initialOrentation]';

robotPosesAndTimestamps = zeros(1,4);

%%


robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
r = robot.WheelRadius;
d = robot.TrackWidth;

figure(1)
plot(road(:,1), road(:,2), 'x')

% Create lidar sensor
lidar = LidarSensor;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-pi/2,pi/2,90);
lidar.maxRange = 3;

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;
viz.mapName = "map";
attachLidarSensor(viz,lidar);

controller = controllerPurePursuit;
controller.Waypoints = road;
controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity    = 2;
controller.LookaheadDistance     = 0.5;



goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%%

% Initialize the simulation loop
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Initialize the figure
figure

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robot.TrackWidth/0.8;

t = 0.1;
iter = 1;
vel_out = [];
angvel_out = [];
Or = [];
Ol = [];
time = [];
time(1) = t;

current_waypoint_idx = 2;
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    

    % Get the robot's velocity using controller inputs
    vel = derivative(robot, robotCurrentPose, [v omega]);

    vel_out    = [vel_out  vel(1:2,:)];
    angvel_out = [angvel_out vel(3,:)]; %#ok<*AGROW> 

    Vr = v + omega*d/2;
    Vl = v - omega*d/2;

    Or = [Or; Vr/r];
    Ol = [Ol; Vl/r];

    [road, changed, current_index_update] = dynamic_RRT_replanning(road, current_waypoint_idx, lidar, robotCurrentPose, controller.MaxAngularVelocity, vel, map_array, scale, d);


    if changed
        current_waypoint_idx = current_waypoint_idx + current_index_update;
        release(controller);
        controller.Waypoints = road;
    end


    % Check if the waypoint has been reached
    if current_waypoint_idx < size(road, 1) && distance(robotCurrentPose(1), robotCurrentPose(2), road(current_waypoint_idx, 1), road(current_waypoint_idx, 2)) < 0.75
        current_waypoint_idx = current_waypoint_idx + 1 % Move to the next waypoint
    end

    % Store previous pose
    robotPosesAndTimestamps(end+1, :) = [robotCurrentPose', time(end)];

    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime; 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:));
    
    % Update the plot
    hold off
        
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    plot(road(:,1), road(:,2),"k--d")
    hold all
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0];
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;

    waitfor(vizRate);
    iter = iter+1;
    time(end+1) = t*iter; %elaped time matrix
    phi = [Or Ol];        %matrix 1st column Right wheel angular velocity 2nd column left

end
    position_time_matrix = robotPosesAndTimestamps(2:end);
end




