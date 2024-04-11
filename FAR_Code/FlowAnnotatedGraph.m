function [G] = FlowAnnotatedGraph(map,gridsize)
%FlowAnnotatedGraph Returns the flow annotated graph to run an A* search on
%   First goes through the every other column/row procedure
%   Then each node is checked to ensure that it has atleast 1 out, 1 in
%   edge
dim = size(map);
num_row = dim(1);
num_col = dim(2);

graphIndex = @(r,c) num_col*(r-1) + c; %Convert to graph node  to number
invGraphIndex = @(ind) [floor((ind-1)/num_col)+1,mod(ind-1,num_col)+1];

source = [];
target = [];
weights = [];

%Add Edges With in the columns
for i =1:num_row
    %If odd Row (Make connection from left to right)
    if(mod(i,2))
        for j = 1:(num_col-1)
            %verify both squares are empty
            if(~map(i,j) && ~map(i,j+1))
                source = [source, graphIndex(i,j)]; %Add left to right edge
                target = [target, graphIndex(i,j+1)];
                weights = [weights,1];
            end
        end        
    else
    %else even Row (Make connection from right to left)
        for j = (num_col):-1:2
            %verify both squares are empty
            if(~map(i,j) && ~map(i,j-1))
                source = [source, graphIndex(i,j)]; %Add right to left edge
                target = [target, graphIndex(i,j-1)];
                weights = [weights,1];
            end
        end        
    end
end

%Add Edges With in the rows
for j =1:num_col
    %If odd column (Make connection from bottom to top)
    if(mod(j,2))
        for i = (num_row):-1:2
            %verify both squares are empty
            if(~map(i,j) && ~map(i-1,j))
                source = [source, graphIndex(i,j)]; %Add bottom to top edge
                target = [target, graphIndex(i-1,j)];
                weights = [weights,1];
            end
        end        
    else
    %else even column (Make connection from top to bottom)
        for i = 1:1:(num_row-1)
            %verify both squares are empty
            if(~map(i,j) && ~map(i+1,j))
                source = [source, graphIndex(i,j)]; %Add top to bottom edge
                target = [target, graphIndex(i+1,j)];
                weights = [weights,1];
            end
        end        
    end
end

blocked_nodes = [];
for i=1:num_row
    for j=1:num_col
        if map(i,j)
            blocked_nodes = [blocked_nodes,graphIndex(i,j)];
        end
    end
end
%Identify inbound and outbound Nodes
indices = [1:1:(num_row*num_col)];

%Nodes remaining are not a source (so connection are only inbound) 
%Need to add an edge coming from these nodes
inbound = setdiff(indices,source); 
inbound = setdiff(inbound,blocked_nodes);
%Nodes remaining are not a target (so connection are only outbound) 
%Need to add an edge going to these nodes
outbound = setdiff(indices,target); 
outbound = setdiff(outbound,blocked_nodes);

%Try adding diagonal connections for inbound/outbound cases
[inSource,inTarget,inWeight]= diagonalEdge(inbound, map, 0);
[outSource,outTarget,outWeight]= diagonalEdge(outbound, map, 1);
source = [source,inSource,outSource];
target = [target,inTarget,outTarget];
weights = [weights,inWeight,outWeight];

%Potential Add ability to ensure connectivity within locality, and 
%automate map ability to handle tunnel (i.e robots have to go through
%single width corridor)- Not hard find Strongly Connected Components(SCCs),
%then find chain of SCCs with only one node to detect a tunnel 
%set up controller to then block/allow one robot at a time through the
%tunnel

%Now add way point to each Graph Node
weights = weights*gridsize;
G = digraph(source,target,weights);
X = zeros(num_row,num_col);
Y = zeros(num_row,num_col);
%Assuming bottom left corner is 0,0 
currentX = 0.5*gridsize;
for i=1:num_col
    X(:,i) = currentX;
    currentX = currentX + gridsize;
end

currentY = 0.5*gridsize;
for j=num_row:-1:1
    Y(j,:) = currentY;
    currentY = currentY + gridsize;
end
X = reshape(X',[],1); %Unravel so that it can be added to graph
Y = reshape(Y',[],1);

G.Nodes.X = X;
G.Nodes.Y = Y;

end


