function plot_robot_paths(robots, fig_num_start)
    figure(fig_num_start)
    for i = 1: length(robots)
        plot(robots(i).pose(1:end, 1), robots(i).pose(1:end, 2))
    end
end
