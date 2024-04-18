%{
    Collision Detection for grid-based environments
    Parameters:
        xNear: The x position that is the nearest node to the xNew
        yNear: The y position that is the nearest node to the yNew
        xNew:  The proposed new x point
        yNew:  The proposed new y point
        map:   The array that contains the loaded csv file 
               for the environment
        scale: The scale of a single grid-tile from the map

    Returns:
        c_test: True if a collision was found, false otherwise
%}

function  c_test = collision_detector(xNear,yNear,xNew, yNew, map, scale)
    % we test 10 uniformally distributed points along the line that
    % connects (xNear,yNear) to (xNew,yNew)
    additional_threshold = 0.5; % Helps keep the position away from the walls better
    s = 0-additional_threshold:0.05:1+additional_threshold;
    xq = xNear + s.*(xNew-xNear);
    yq = yNear + s.*(yNew-yNear);
    
    c_test = 0; % initialize as false (no collision)
        
        % Check for collisions with the map
        for k = 1:length(xq)
            grid_x = floor(xq(k) / scale) + 1;
            grid_y = floor((size(map,1) - floor(yq(k)) / scale));
            
            % Check if the grid cell contains an obstacle or if it is
            % outside of the boundaries
            if (grid_x < 1 || grid_x >= size(map,2) || grid_y < 1 || grid_y >= size(map,1))
                c_test = 1; % Collision detected
                return;
            else if map(grid_y, grid_x) == 1
                c_test = 1; % Collision detected
                return;
            end
        end
    end