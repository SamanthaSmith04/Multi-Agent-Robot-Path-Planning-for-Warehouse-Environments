function [newSource,newTarget,newWeights] = diagonalEdge(nodes, map, direction)
    %Use direction = 0 to add diagonal for inbound edges, and 1 for
    %outbound
    dim = size(map);
    num_row = dim(1);
    num_col = dim(2);
    graphIndex = @(r,c) num_col*(r-1) + c; %Convert to graph node  to number
    invGraphIndex = @(ind) [floor((ind-1)/num_col)+1,mod(ind-1,num_col)+1];
    newSource = [];
    newTarget = [];
    
    
    for node = nodes
        coo = invGraphIndex(node);
        r = coo(1);
        c = coo(2);
        % 4 Diagonal Edges to try (NE, NW, SE, SW)
        % Create a 3x3 local matrix to determine best way to create a diagonal
        local = zeros(3);
        if(r-1 < 1)
            local(1,:) = 1;
        elseif (r+1 > num_row)
            local(3,:) = 1;
        end

        if(c-1 < 1)
            local(:,1) = 1;
        elseif (c+1 > num_col)
            local(:,3) = 1;
        end

        for i = [-1, 0, 1]
            for j = [-1, 0, 1]
                % Not already been checked
                % Also ensure within bounds
                if(~local(i+2,j+2))
                    local(i+2,j+2) = map(r+i,c+j);
                end
            end
        end

        % Goal Find a corner where all top/bottom and left/right are open 
        local(1,1) = local(1,1)+local(1,2)+local(2,1);
        local(1,3) = local(1,3)+local(1,2)+local(2,3);
        local(3,1) = local(3,1)+local(3,2)+local(2,1);
        local(3,3) = local(3,3)+local(3,2)+local(2,3);

        %At this point if a corner of local equals 0 we can add an edge to it
        if(~local(1,1))
            newSource = [newSource, node];
            newTarget = [newTarget, graphIndex(r-1,c-1)];
        elseif (~local(1,3))
            newSource = [newSource, node];
            newTarget = [newTarget, graphIndex(r-1,c+1)];
        elseif (~local(3,1))
            newSource = [newSource, node];
            newTarget = [newTarget, graphIndex(r+1,c-1)];
        elseif (~local(3,3))
            newSource = [newSource, node];
            newTarget = [newTarget, graphIndex(r+1,c+1)];
        end
    end
    
    % For outbound swap target and source nodes
    if direction
        temp = newSource;
        newSource=newTarget;
        newTarget=temp;
    end
    
    numAdditions = size(newSource);
    newWeights = zeros(1,numAdditions(2))*sqrt(2);
end