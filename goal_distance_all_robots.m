%{
    Restructures the distances of all robots from the goal into a single
    array
    Parameters: 
        robots_array: The array that holds the robot structs
    Returns: The distances of all robots from their goal
%}

function distances_to_goal = goal_distance_all_robots(robots_array) 
    distances_to_goal = [];
    for i=1: numel(robots_array)
        distances_to_goal(i) = robots_array(i).distanceToGoal;
    end
end