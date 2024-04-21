function [] = write_exp_struct(simulated_robots,outputStructFileName)
%WRITE_EXP_STRUCT Summary of this function goes here
%   Detailed explanation goes here

extraPrecision = 100000;
[dist, avgDist, stdDist] = distanceBetweenRobot(simulated_robots);
experiment_struct.endTime = simulated_robots.endTime;
experiment_struct.path_length = simulated_robots.path_length;
experiment_struct.avg_dist = avgDist;
experiment_struct.std_dist = stdDist;

temp = reshape(dist,1,[])*extraPrecision;
experiment_struct.distances = temp;
experiment_struct.dist_size = size(dist);
experiment_struct.time = simulated_robots(1).time;
writestruct(experiment_struct, outputStructFileName);
end

