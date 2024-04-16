function distances_to_goal = goal_distance_all_robots(robots_array) 
    distances_to_goal = [];
    for i=1: numel(robots_array)
        distances_to_goal(i) = robots_array(i).distanceToGoal;
    end
    distances_to_goal
end