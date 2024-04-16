function [phi, time, robotPoses] = FARmotionPlan(road, startPose, sampleTime)

%phi is the angular velocity of each of the wheels at each instance of
%sampletime
%time is the overall time vector for the planning

%road is the waypoints of the path planned from the algorithms
%thetaInit is the initial angle of the robot from the x axis

robotInitialLocation = startPose(1:2);
robotGoal            = road(end,:);
robotCurrentPose     = startPose';

%%

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
r = robot.WheelRadius;
d = robot.TrackWidth;

controller = controllerPurePursuit;
controller.Waypoints = [robotInitialLocation;road];

controller.DesiredLinearVelocity = 1;
controller.MaxAngularVelocity    = 2;
controller.LookaheadDistance     = 0.5;

goalRadius = 0.1;
distanceToGoal = norm(robotInitialLocation - robotGoal);

%%

% Initialize the simulation loop
vizRate = rateControl(1/sampleTime);


t = sampleTime;
iter = 1;
vel_out = [];
angvel_out = [];
Or = [];
Ol = [];
time = [];

robotPoses = []; 

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
    
    % Plot the path of the robot as a set of transforms    
    time(end+1) = t*iter; %elaped time matrix
    iter = iter+1;
    phi = [Or Ol];        %matrix 1st column Right wheel angular velocity 2nd column left
    robotPoses = [robotPoses;robotCurrentPose'];

end

end




