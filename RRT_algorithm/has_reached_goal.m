function reached = has_reached_goal(current_position, goal_position)
    threshold = 1;
    if distance(current_position(1), current_position(2), goal_position(1), goal_position(2))< threshold
        reached = true;
    else
        reached = false;
    end
end
