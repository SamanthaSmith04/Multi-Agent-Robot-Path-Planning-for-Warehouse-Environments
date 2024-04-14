function [node_path] = AstarDigraph(G,start,target,map,grid_size,start_pos,goal_pos,fig_num)
%Astar Digraph - For a given digraph it finds the shortest path
%   G - The digraph being search
%   Map - A map of node number to coordinates of node
%   start - node number number corresponding to start point
%   target - set of nodes with in radius of end point
%   return: node_path = [start, ..., x \in target] 
%               (...: respresnts node numbers in between)
viz = 0;
if(nargin > 3)
    viz = 1;
end

N = numnodes(G);
open = [start];
closed = []; % Empty for now
pastCost = Inf(1,N); % Past Costs
estCost = Inf(1,N);
pastCost(start) = 0;
estCost(start) = heuristic(G,start,target); % TODO Write up heuristic function

parent = zeros(1,N); % Stores the parent node for each node

scale = 1;
if  viz
    scale = grid_size;
    bias = 1;
    max_dist = scale;
    max_tree = 2000;
    goal_threshold = scale;

    %% set up bounds for planning
    x_min = 0;
    y_min = 0; 
    map_dims = size(map);
    x_max = (map_dims(2)) * scale;
    y_max = (map_dims(1)) * scale;
    


    %% plot the initial RRT plan graph
    figure(fig_num); hold on; grid on
    
    axis equal
    axis([x_min,x_max,y_min,y_max]);
    % plot the boundary
    plot([x_min, x_min, x_max, x_max, x_min],[y_min, y_max, y_max, y_min, y_min], 'k--', 'LineWidth', 1); 
    drawnow;
    % plot the starting point
    plot(start_pos(1), start_pos(2), 'ko', 'MarkerSize',3, 'MarkerFaceColor','k');
    drawnow;
    % plot the goal point and goal region
    plot(goal_pos(1), goal_pos(2), 'go', 'MarkerSize',3, 'MarkerFaceColor','g');
    drawnow;
    th = 0:pi/50:2*pi;
    xcircle = goal_threshold * cos(th) + goal_pos(1);
    ycircle = goal_threshold * sin(th) + goal_pos(2);
    h = plot(xcircle, ycircle);
    drawnow;
    % Plot the obstacles
    for i = 1:map_dims(1) %rows
        for j = 1:map_dims(2) %cols
            if map(i, j) == 1
                rectangle('Position', [(j-1)*scale, (map_dims(1)-i)*scale, scale, scale], 'FaceColor', 'r', 'EdgeColor', 'none');
                drawnow;
            end
        end
    end
end

%Debugging to see all connections in the graph
%     for i = 1:N
%         neigh = successors(G,i)';
%         for j = neigh
%             X_coord = G.Nodes.X([i,j]);
%             Y_coord = G.Nodes.Y([i,j]);
%             plot(X_coord,Y_coord, 'b');
%             pause(0.001);
% 
%         end
%     end





% While Open is non-empty
while size(open) > 0
    current = open(1); % Get first node in open
    open = open(2:end); % Remove the first element
    closed(end+1) = current; %Add current to closed
    if ismember(current, [target])
        node_path = [current];
        %As long as the node has a parent add it into the path
        %otherwise stop since presumable the first node is now start node
        while parent(current)
            current = parent(current);
            node_path = [current,node_path]; %#ok<*SAGROW>
        end
        
        if(viz)
        X_coord = G.Nodes.X(node_path);
        Y_coord = G.Nodes.Y(node_path);

        plot([X_coord(1)],[Y_coord(1)], 'g');
        for i = 2:size(X_coord)
        plot([X_coord(i-1); X_coord(i)],[Y_coord(i-1); Y_coord(i)], 'g');
            pause(0.02);
        end
        end
        
        return;
    end
    neighbors = setdiff(successors(G,current)',closed);
    for nbr = neighbors
        cost = G.Edges.Weight(findedge(G,current,nbr));
        tentativePastCost = pastCost(current) + cost;
        if tentativePastCost < pastCost(nbr)
            pastCost(nbr) = tentativePastCost;
            parent(nbr) = current;
            estCost(nbr) = pastCost(nbr) + heuristic(G,nbr,target);
            open = setdiff(open,[nbr]); %remove neighbor from set if needed
            % Add to open in a sorted fashion relative to estCost
            sizeOpen = size(open);
            sizeOpen = sizeOpen(2);
            insert = 1;
            compare = estCost(nbr) > estCost(open);
            open = [open(compare),nbr,open(~compare)];
            
            %Viz
            if(viz)
            X_coord = G.Nodes.X([parent(nbr),nbr]);
            Y_coord = G.Nodes.Y([parent(nbr),nbr]);
            plot(X_coord,Y_coord, 'b');
            pause(0.001);
            end
        end
    end
end

node_path = 0; % failure
