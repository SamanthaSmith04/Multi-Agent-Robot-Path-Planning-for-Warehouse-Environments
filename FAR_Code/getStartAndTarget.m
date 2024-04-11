function [startNode,goalNode] = getStartAndTarget(map,gridsize,start,goal)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

%Assume coordinates, are within boundaries
dim = size(map);
num_row = dim(1);
num_col = dim(2);

graphIndex = @(r,c) num_col*(r-1) + c; %Convert to graph node  to number
invGraphIndex = @(ind) [floor((ind-1)/num_col)+1,mod(ind-1,num_col)+1];

startX = start(1);
startY = start(2);
goalX = goal(1);
goalY = goal(2);


sC = ceil(startX/gridsize);
gC = ceil(goalX/gridsize);
sR = num_row - floor(startY/gridsize);
gR = num_row - floor(goalY/gridsize);
startNode = graphIndex(sR,sC);
goalNode = graphIndex(gR,gC);
end

