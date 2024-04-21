function [done] = vizualizeFARmotion3(simulated_robots,timeStep,map,grid_size,figureNumber)
%Visualize Upto 3 Robots
%   Detailed explanation goes here

if nargin < 5
    figureNumber = 1;
end


figure(figureNumber)

pause(1)
scale = grid_size;
x_min = 0-0.5;
y_min = 0-0.5; 
map_dims = size(map);
x_max = 0.5+ (map_dims(2)) * scale;
y_max = 0.5+ (map_dims(1)) * scale;

axis equal
axis([x_min,x_max,y_min,y_max]);
% plot the boundary
plot([x_min, x_min, x_max, x_max, x_min],[y_min, y_max, y_max, y_min, y_min], 'k--', 'LineWidth', 1); 
% Plot the obstacles
for i = 1:map_dims(1) %rows
    for j = 1:map_dims(2) %cols
        if map(i, j) == 1
            rectangle('Position', [(j-1)*scale, (map_dims(1)-i)*scale, scale, scale], 'FaceColor', 'r', 'EdgeColor', 'none');
            
        end
    end
end

hold on;
inital_path_plots = ['k--','b--','g--'];
color_strings = ["black","blue","green"];
num_bots = length(simulated_robots);
for i = 1:num_bots
   road = simulated_robots(i).road;
   plot(road(:,1), road(:,2), inital_path_plots(i));
end
hold off;

%Animate Robot Movement

robot = differentialDriveKinematics("TrackWidth", 1, "VehicleInputs", "VehicleSpeedHeadingRate");
frameSize = robot.TrackWidth/0.8;



vizRate = rateControl(1/(timeStep));
%Assume All Robots have the same number of time stamps
numSteps = length(simulated_robots(1).time);
for i = 1:5:numSteps
    hold off;
    
    %Redraw Obstacles
    axis equal
    axis([x_min,x_max,y_min,y_max]);
    % plot the boundary
    plot([x_min, x_min, x_max, x_max, x_min],[y_min, y_max, y_max, y_min, y_min], 'k--', 'LineWidth', 1); 
    % Plot the obstacles
    for k = 1:map_dims(1) %rows
        for j = 1:map_dims(2) %cols
            if map(k, j) == 1
                rectangle('Position', [(j-1)*scale, (map_dims(1)-k)*scale, scale, scale], 'FaceColor', 'r', 'EdgeColor', 'none');

            end
        end
    end
    
    hold on
    
    %Plot the roads
    for j = 1:num_bots
        road = simulated_robots(j).road;
        plot(road(:,1), road(:,2), inital_path_plots(j)) 
    end
    
    for j=1:num_bots
        robotCurrentPose = simulated_robots(j).pose(i,:);
        % Plot the path of the robot as a set of transforms
        plotTrVec = [robotCurrentPose(1:2)'; 0];
        plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize, "MeshColor", color_strings(j));
        light;
        drawnow;
    end
    
    waitfor(vizRate);

end

done = 1;
end

