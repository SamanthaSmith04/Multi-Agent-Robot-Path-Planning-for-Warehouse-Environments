function [distance] = heuristic(G,node,target)
%Heuristic function is a manhattan distance (not euclidean since diagonal
%is not allowed
%   Detailed explanation goes here

x = G.Nodes.X;
y = G.Nodes.Y;
% disp(node)
% disp(target)
distance = abs(x(node)-x(target))+abs(y(node)-y(target));
end

