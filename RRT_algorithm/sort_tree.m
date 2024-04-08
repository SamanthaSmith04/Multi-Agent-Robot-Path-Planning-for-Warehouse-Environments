function index = sort_tree(nodes, xRand, yRand)

    dist = Inf*ones(1,length(nodes));
    for j = 1:length(nodes)
        dist(j) = distance(xRand,yRand,nodes(j).x,nodes(j).y);
    end
    [~, index] = min(dist);
end

