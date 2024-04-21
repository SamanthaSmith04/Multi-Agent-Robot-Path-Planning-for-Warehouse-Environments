function [distance_robots] = pathLength(robots)
%UNTITLED Apprxoimates the path length by linearization
%   Sums up the distance travelled in each time step to formulate
%   a heurisitc output

for i = 1:length(robots)
    pose_i = robots(i).pose;
    path_i = 0;
    for j = 2:length(pose_i)
        travelled = pose_i(j,1:2) - pose_i(j-1,1:2);
        travelled = sum(travelled.^2);
        path_i = path_i + sqrt(travelled);
    end
    robots(i).path_length = path_i;
end

distance_robots = robots;
end

