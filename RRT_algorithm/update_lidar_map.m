%{
    Updates the occupacy map for the environment so other robots can detect
    the robot
    Parameters:
        map: The occupacy map of the environment
        robot: The robot to update the map at it's position
        map_array: The matrix representation of the environment
        map_array_original: The initial environment (no robots)
        value: The value to update the occupied grids to

    Returns:
        oc_map: The updated occupacy map
        updated_array: The updated map_array
%}

function [oc_map, updated_array] = update_lidar_map(map, robot, map_array, map_array_original, value)
    oc_map = map;
    updated_array = map_array;
    x = robot.pose(end, 1);
    y = robot.pose(end, 2);

    x_ceil = ceil(x);
    x_floor = floor(x);

    y_ceil = size(map_array_original, 1) - floor(y);
    y_floor = size(map_array_original, 1) - ceil(y);

    for x_grid = x_floor : x_ceil
        for y_grid = y_floor : y_ceil
            if (x_grid > 0 && y_grid > 0 && ...
                    x_grid <= size(map_array_original, 2) && y_grid <= size(map_array_original, 1) && ...
                    map_array_original(y_grid, x_grid) ~= 1) 
                oc_map(y_grid, x_grid) = value;
                updated_array(y_grid, x_grid) = value;

            end
        end
    end

end
