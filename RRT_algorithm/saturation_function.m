function sat = saturation_function(x, upper_limit, T)
    if (abs(x) < upper_limit*T)
        sat =  x;
    else 
        sat = upper_limit*T;
    end
end