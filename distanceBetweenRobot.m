function [distance, avgDistance, stdDistance] = distanceBetweenRobot(simulated_robots)
%Returns the Distance and Average distance between each robot at each time
%step
%   Format For distance between each robot and how it is reported
%   Distance = [1->2, 1->3, ..., 1-n, 2->3,2->4, ..., 2->n, 3->4, 3->5, ...
%   3->n, ..., ..., n-1->n]

num_robots = length(simulated_robots);
distance = [];
for i = 1:num_robots;
    for j = i+1:num_robots;
        bot_i_coo = simulated_robots(i).pose(:,1:2);
        bot_j_coo = simulated_robots(j).pose(:,1:2);
        
        %Compute Euclidean Distance
        dist = bot_i_coo-bot_j_coo;
        dist = dist.^2;
        dist = sqrt(sum(dist, 2));
        distance = [distance, dist];
        
    end
end

%Computed for each column, thus dim keyword is 1
avgDistance = mean(distance, 1);
stdDistance = std(distance, 0 ,1); 
%Unbiased standard deviation, (divide by n-1)
end

