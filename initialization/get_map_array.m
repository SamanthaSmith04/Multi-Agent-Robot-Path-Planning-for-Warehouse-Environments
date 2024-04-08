%{
    Retrieves the matrix of the map contained within the CSV file
    Parameters: 
        csv_file: The path to the CSV file to read from
                  (0 for empty spaces, 1 for obstacles)
    Returns:
        csv_matrix: The matrix form of the CSV file
    
%} 

function csv_matrix = get_map_array(csv_file)
    csv_matrix = readmatrix(csv_file);
end