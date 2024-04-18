function [updated_plan, changed_plan, current_index_update] = dynamic_RRT_replanning(plan, current_plan_step, lidar, current_pose, uM, current_velocity, map, scale, L, sampling_interval)
   updated_plan = plan;
   changed_plan = false;
   safety_region_size = 1.5; % Safety region size in units
   current_index_update = 0;
   deletion_threshold = 1.5;
   T = sampling_interval; % Sampling interval in seconds

   % Check if safety region is empty
   ranges = lidar(current_pose);
   if any(ranges < safety_region_size)
       % Calculate deviation angle C(t) between current pose and waypoint
       theta_t = atan2(plan(current_plan_step, 2) - current_pose(2), plan(current_plan_step, 1) - current_pose(1));
       theta = current_pose(3);
       num_waypoints = size(plan, 1);
    % find the angle between the robot and all waypoints
    for i = 1:num_waypoints
        angle_to_waypoint(i) = atan2(plan(i, 2) - current_pose(2), plan(i, 1) - current_pose(1));
    end
    min_angle_diff = abs(theta_t - angle_to_waypoint(1)); 
    
    for i = 2:num_waypoints
        angle_diff = abs(theta_t - angle_to_waypoint(i));
        if angle_diff < min_angle_diff
            closest_waypoint_index = i;
            min_angle_diff = angle_diff;
        end
    end

        
       Aj_minus = angle_to_waypoint(closest_waypoint_index);
       Aj_plus = angle_to_waypoint(closest_waypoint_index + 1);
       Aj = (Aj_minus + Aj_plus) / 2;
%        Csat = saturation_function(Aj - theta, uM, T);
       Csat = Aj;
       % Calculate total velocity
       v = sqrt(current_velocity(1)^2 + current_velocity(2)^2);
       new_path_point = [current_pose(1) + v * cos(Csat), current_pose(2) + v * sin(Csat)];
       distance_between_new_and_current = norm(new_path_point - current_pose(1:2)');
       distance_between_current_and_next_original = norm(plan(current_plan_step, :) - current_pose(1:2)');
       
       % Check collision and update plan
       if ~collision_detector(current_pose(1), current_pose(2), new_path_point(1), new_path_point(2), map, scale)
           if current_plan_step+1 < size(plan,1) && distance_between_current_and_next_original >= distance_between_new_and_current && distance_between_current_and_next_original - distance_between_new_and_current <= deletion_threshold
               updated_plan = [plan(1:current_plan_step-1, :); new_path_point; plan(current_plan_step+1:end, :)];
               %disp("CASE 1")
               current_index_update = -1;
           else
               updated_plan = [plan(1:current_plan_step-1, :); new_path_point; plan(current_plan_step:end, :)];
               %disp("CASE 2")
           end
           changed_plan = true;
           current_index_update = 1;
       end
   end
end