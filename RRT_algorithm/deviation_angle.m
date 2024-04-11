function angle = deviation_angle(current_pose, waypoint_position, index) 
    angle = atan2(waypoint_position(2) - current_pose(2), waypoint_position(1) - current_pose(1)) - current_pose(3);
end