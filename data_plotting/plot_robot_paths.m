function plot_robot_paths(robots, fig_num_start, map_array)
    
    path_plots = {'k--','b--','g--'};
    motion_plots = {'k-','b-','g-'};
    points = {'ko', 'bo', 'go'};

    % Plot motion vs planned path
    figure(fig_num_start)
    xlim([0, size(map_array,2)]);
    ylim([0, size(map_array,1)]);
    hold on;
    for i = 1: length(robots)
        plot(robots(i).pose(:, 1), robots(i).pose(:, 2), motion_plots{i});
        plot(robots(i).road(:, 1), robots(i).road(:, 2), path_plots{i});

        plot(robots(i).pose(1, 1), robots(i).pose(1, 2), points{i}, 'MarkerSize', 10); % Start point
        plot(robots(i).pose(end, 1), robots(i).pose(end, 2), points{i}, 'MarkerSize', 10); % End point
    end
    hold off;

    % Add titles and labels
    title('Motion vs Planned Path');
    xlabel('X-axis');
    ylabel('Y-axis');
    legend('Robot 1 Motion', 'Robot 1 Planned Path', 'Robot 1 Start Position', 'Robot 1 Goal Position', ...
           'Robot 2 Motion', 'Robot 2 Planned Path', 'Robot 2 Start Position', 'Robot 2 Goal Position', ...
           'Robot 3 Motion', 'Robot 3 Planned Path', 'Robot 3 Start Position', 'Robot 3 Goal Position');

    % Plot original road vs updated road
    for i = 1: length(robots)
        fig_num_start = fig_num_start +1;
        figure(fig_num_start)
        hold on;
        plot(robots(i).original_road(:, 1), robots(i).original_road(:, 2), "r--");
        plot(robots(i).road(:, 1), robots(i).road(:, 2), "b-");
        title(['Robot ', num2str(i), ' Original Road vs Updated Road']);
        xlabel('X-axis');
        ylabel('Y-axis');
        legend('Original Road', 'Updated Road');
        hold off;
    end
end
