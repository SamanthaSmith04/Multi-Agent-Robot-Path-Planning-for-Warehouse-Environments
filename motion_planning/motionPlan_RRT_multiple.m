function [phi, time, robots_array] = motionPlan_RRT_multiple(robots_array, map, map_original_array, scale, visual, speed)

%phi is the angular velocity of each of the wheels at each instance of
%sampletime
%time is the overall time vector for the planning
%position_time_matrix is the position and timestamp of each robot pose at
%each sample time
%road is the road of the path planned from the algorithms
%thetaInit is the initial angle of the robot from the x axis
num_robots = numel(robots_array);
map_array = map_original_array;
for i=1:num_robots 
    robots_array(i).robotInitialLocation = robots_array(i).road(1,:);
    robots_array(i).robotGoal            = robots_array(i).road(end,:);
    
    robots_array(i).initialOrentation    = robots_array(i).thetaInit;
    
    robots_array(i).pose     = [robots_array(i).robotInitialLocation robots_array(i).initialOrentation];
    
    robots_array(i).robotPosesAndTimestamps = zeros(1,4);
    
    robots_array(i).robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
    robots_array(i).r = robots_array(i).robot.WheelRadius;
    robots_array(i).d = robots_array(i).robot.TrackWidth;
    
    
    % Create lidar sensor
    robots_array(i).lidar = LidarSensor;
    robots_array(i).lidar.sensorOffset = [0,0];
    robots_array(i).lidar.scanAngles = linspace(-pi/2,pi/2,50);
    robots_array(i).lidar.maxRange = 5;
    
    % Create visualizer
    robots_array(i).viz = Visualizer2D;
    robots_array(i).viz.hasWaypoints = true;
    robots_array(i).viz.mapName = "map";
    attachLidarSensor(robots_array(i).viz, robots_array(i).lidar);
    
    robots_array(i).controller = controllerPurePursuit;
    robots_array(i).controller.Waypoints = robots_array(i).road;
    robots_array(i).controller.DesiredLinearVelocity = 1;
    robots_array(i).controller.MaxAngularVelocity    = 2;
    robots_array(i).controller.LookaheadDistance     = 0.5;
    
    
    
    robots_array(i).distanceToGoal = norm(robots_array(i).robotInitialLocation - robots_array(i).robotGoal);

    robots_array(i).Or = [];
    robots_array(i).Ol = [];

    robots_array(i).current_waypoint_idx = 2;
    robots_array(i).time = 0;

end
goalRadius = 0.3;

%%

% Initialize the simulation loop
sampleTime = 0.1;
% Initialize the figure


% Determine vehicle frame size to most closely represent vehicle with plotTransforms

t = 0.1;
iter = 1;
vel_out = [];
angvel_out = [];

time = [];
time(1) = t;

distances = goal_distance_all_robots(robots_array);
while( any(distances > goalRadius))

    for i=1:num_robots
        if (distances(i) <= goalRadius) 
            robots_array(i).pose = [robots_array(i).pose; robots_array(i).pose(end, :)]; 
            continue;
        end     
        [map, map_array] = update_lidar_map(map, robots_array(i), map_array, map_original_array, 0, scale);

        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = robots_array(i).controller(robots_array(i).pose(end, :)');        
    
        % Get the robot's velocity using controller inputs
        vel = derivative(robots_array(i).robot, robots_array(i).pose(end, :), [v omega]);
    
        vel_out    = [vel_out  vel(1:2,:)];
        angvel_out = [angvel_out vel(3,:)]; %#ok<*AGROW> 
    
        Vr = v + omega*robots_array(i).d/2;
        Vl = v - omega*robots_array(i).d/2;
    
        robots_array(i).Or = [robots_array(i).Or; Vr/robots_array(i).r];
        robots_array(i).Ol = [robots_array(i).Ol; Vl/robots_array(i).r];
        
        if (robots_array(i).current_waypoint_idx < size(robots_array(i).road, 1))
            [robots_array(i).road, changed, current_index_update] = dynamic_RRT_replanning(robots_array(i).road, ...
            robots_array(i).current_waypoint_idx, robots_array(i).lidar, robots_array(i).pose(end,:)', ...
            robots_array(i).controller.MaxAngularVelocity, vel, map_array, scale, robots_array(i).d, sampleTime);
        end
    
        if changed
            robots_array(i).current_waypoint_idx = robots_array(i).current_waypoint_idx + current_index_update;
            release(robots_array(i).controller);
            robots_array(i).controller.Waypoints = robots_array(i).road;
        end
    
    
        % Check if the waypoint has been reached
        if robots_array(i).current_waypoint_idx < size(robots_array(i).road, 1) && ...
            distance(robots_array(i).pose(end, 1), robots_array(i).pose(end, 2), ...
            robots_array(i).road(robots_array(i).current_waypoint_idx, 1), ...
            robots_array(i).road(robots_array(i).current_waypoint_idx, 2)) < 0.75
            robots_array(i).current_waypoint_idx = robots_array(i).current_waypoint_idx + 1; % Move to the next waypoint
        end
    
        % Store previous pose
        robots_array(i).robotPosesAndTimestamps(end+1, :) = [robots_array(i).pose(end, :), time(end)];
    
        % Update the current pose
        newPose = robots_array(i).pose(end, :) + (vel*sampleTime)';
        robots_array(i).pose = [robots_array(i).pose; newPose]; 
        
        % Re-compute the distance to the goal
        robots_array(i).distanceToGoal = distance(newPose(1), newPose(2), robots_array(i).robotGoal(1), robots_array(i).robotGoal(2));
        robots_array(i).phi = [robots_array(i).Or robots_array(i).Ol];        %matrix 1st column Right wheel angular velocity 2nd column left
        robots_array(i).time = robots_array(i).time + t;

        [map, map_array] = update_lidar_map(map, robots_array(i), map_array, map_original_array, 1, scale);

    end
        if visual && mod(iter, speed) == 0
            visualizeRRTmotion3(robots_array, t, map_original_array, scale,2);
        end
        iter = iter+1;
        time(end+1) = t*iter; %elaped time matrix
        distances = goal_distance_all_robots(robots_array);
        

end
    phi = zeros(2)
    
    plot_robot_paths(robots_array, 3, map_original_array, scale)
    for i = 1: size(robots_array)
        robots_array(i).endTime = robots_array(i).time(end);
    end 
end




