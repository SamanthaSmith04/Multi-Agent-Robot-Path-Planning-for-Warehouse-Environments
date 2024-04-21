function [] = write_exp_struct(simulated_robots,outputStructFileName)
%WRITE_EXP_STRUCT Summary of this function goes here
%   Detailed explanation goes here

extraPrecision = 100000;
[dist, avgDist, stdDist] = distanceBetweenRobot(simulated_robots);
endTimesAll = [];
lengthAll = [];
for i = 1:length(simulated_robots)
    endTimesAll = [endTimesAll,simulated_robots(i).endTime];
    lengthAll = [lengthAll,simulated_robots(i).path_length];
end
experiment_struct.endTime = endTimesAll;
experiment_struct.path_length = lengthAll;
experiment_struct.avg_dist = avgDist;
experiment_struct.std_dist = stdDist;

temp = reshape(dist,1,[])*extraPrecision;
experiment_struct.distances = temp;
experiment_struct.dist_size = size(dist);
experiment_struct.time = simulated_robots(1).time;
writestruct(experiment_struct, outputStructFileName);
end

