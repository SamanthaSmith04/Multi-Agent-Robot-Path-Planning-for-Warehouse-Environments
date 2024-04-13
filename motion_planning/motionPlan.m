function [phi, time] = motionPlan(road, thetaInit)

%phi is the angular velocity of each of the wheels at each instance of
%sampletime
%time is the overall time vector for the planning

%road is the waypoints of the path planned from the algorithms
%thetaInit is the initial angle of the robot from the x axis
robotInitialLocation = road(1,:);
robotGoal            = road(end,:);

initialOrentation    = thetaInit;

robotCurrentPose     = [robotInitialLocation initialOrentation]';

%%

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
r = robot.WheelRadius;
d = robot.TrackWidth;

figure(1)
plot(road(:,1), road(:,2), 'x')

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

end




