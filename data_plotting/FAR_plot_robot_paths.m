%{
    Plots the data for robot motion, original path plan, and updated path
    plan

    Parameters:
        robots: the array of robot structs
        fig_num_start: The figure number to begin with
        map_array: the matrix of the evironment
%}

function FAR_plot_robot_paths(robots, fig_num_start, map_array, scale)
    
    path_plots = {'k--','b--','g--'};
    motion_plots = {'k-','b-','g-'};
    points = {'ko', 'bo', 'go'};
    goal_points = {'k*', 'b*', 'g*'};

    %% Plot motion vs planned path
    figure(fig_num_start)
    xlim([0, size(map_array,2)*scale]);
    ylim([0, size(map_array,1)*scale]);
    hold on;
    for i = 1: length(robots)
        plot(robots(i).pose(:, 1), robots(i).pose(:, 2), motion_plots{i});
        plot(robots(i).road(:, 1), robots(i).road(:, 2), path_plots{i});

        plot(robots(i).pose(1, 1), robots(i).pose(1, 2), points{i}, 'MarkerSize', 10); % Start point
        plot(robots(i).pose(end, 1), robots(i).pose(end, 2), goal_points{i}, 'MarkerSize', 10); % End point
    end
    title('Robot Motion vs Planned Path');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Robot 1 Motion', 'Robot 1 Planned Path', 'Robot 1 Start Position', 'Robot 1 Goal Position', ...
           'Robot 2 Motion', 'Robot 2 Planned Path', 'Robot 2 Start Position', 'Robot 2 Goal Position', ...
           'Robot 3 Motion', 'Robot 3 Planned Path', 'Robot 3 Start Position', 'Robot 3 Goal Position');
    hold off;

    %% Plot motion in environment
    fig_num_start = fig_num_start +1;
    figure(fig_num_start)
    figure(fig_num_start)
    xlim([0, size(map_array,2)*scale]);
    ylim([0, size(map_array,1)*scale]);

    x_min = 0;
    y_min = 0;
    x_max = size(map_array,2)*scale;
    y_max = size(map_array,1)*scale;

    map_dims = size(map_array);
    hold on;

    % plot the boundary
    plot([x_min, x_min, x_max, x_max, x_min],[y_min, y_max, y_max, y_min, y_min], 'k--', 'LineWidth', 1); 
    % Plot the obstacles
    for i = 1:map_dims(1) %rows
        for j = 1:map_dims(2) %cols
            if map_array(i, j) == 1
                rectangle('Position', [(j-1)*scale, (map_dims(1)-i)*scale, scale, scale], 'FaceColor', 'r', 'EdgeColor', 'none');
                
            end
        end
    end
    for i = 1: length(robots)
        plot(robots(i).pose(:, 1), robots(i).pose(:, 2), motion_plots{i});

        plot(robots(i).pose(1, 1), robots(i).pose(1, 2), points{i}, 'MarkerSize', 10); % Start point
        plot(robots(i).pose(end, 1), robots(i).pose(end, 2), goal_points{i}, 'MarkerSize', 10); % End point
    end
    title('Robot Motion in Environment');
    xlabel('X Position');
    ylabel('Y Position');
    legend('Robot 1 Motion', 'Robot 1 Start Position', 'Robot 1 Goal Position', ...
           'Robot 2 Motion', 'Robot 2 Start Position', 'Robot 2 Goal Position', ...
           'Robot 3 Motion', 'Robot 3 Start Position', 'Robot 3 Goal Position');
    hold off;

%     %% Plot original road vs updated road
%     for i = 1: length(robots)
%         fig_num_start = fig_num_start +1;
%         figure(fig_num_start)
%         hold on;
%         xlim([0, size(map_array,2)*scale]);
%         ylim([0, size(map_array,1)*scale]);
%         plot(robots(i).original_road(:, 1), robots(i).original_road(:, 2), "r--");
%         plot(robots(i).original_road(1, 1), original_road(i).pose(1, 2), "ro", 'MarkerSize', 10); % Start point
%         plot(robots(i).road(:, 1), robots(i).road(:, 2), "b-");
%         plot(robots(i).original_road(end, 1), original_road(i).pose(end, 2), "bo", 'MarkerSize', 10); % Start point
%         title(['Robot ', num2str(i), ' Original Path Plan vs Updated Path Plan']);
%         xlabel('X Position');
%         ylabel('Y Position');
%         legend('Original Road', 'Updated Road', 'Goal End', 'Actual End');
%         hold off;
%     end
end
