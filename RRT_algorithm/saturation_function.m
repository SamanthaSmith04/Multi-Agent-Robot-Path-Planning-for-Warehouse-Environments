function sat = saturation_function(x, upper_limit, T)

    if (abs(x) <= upper_limit/T)
        sat =  x;
    else 
        sat = sign(x) * upper_limit/T;
    end
end