function [xNew, yNew] = local_planner(xNear,yNear,xRand,yRand,MaxDist)
    dist = distance(xRand,yRand,xNear,yNear);
    % if the distance to go to the (xRand,yRand) is greater than the
    % maximum distance, then travel MaxDist toward (xRand,yRand)
    if dist > MaxDist
        xNew = xNear + MaxDist*(xRand-xNear)/dist;
        yNew = yNear + MaxDist*(yRand-yNear)/dist;
    else % otherwise, travel to the (xRand,yRand)
        xNew = xRand;
        yNew = yRand;
    end
end