%{
    Gets the start and end goals of a trial for all robots from a CSV file
    Parameters:
        csv_file: The CSV file to read from
    Return: The start and end positions
%}

function start_and_goals = get_position_and_goals(csv_file) 
    start_and_goals = readmatrix(csv_file);

end