function start_and_goals = get_position_and_goals(csv_file) 
    start_and_goals = readmatrix(csv_file);

    start_and_goals = start_and_goals(2:end, :);

end