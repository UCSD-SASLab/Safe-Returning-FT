function shortest_Dist = compute_shortest_Dist_2D_poly(obs_map, senseRange, position)
    d_list = nan(1, size(obs_map.xv,1));
    for i = 1:size(obs_map.xv,1)
        [d, ~] = p_poly_dist(position(1), position(2),...
                    obs_map.xv(i,:), obs_map.yv(i,:));
        d_list(i) = d;
    end
    shortest_Dist = min(d_list);
    if shortest_Dist> senseRange
        shortest_Dist = senseRange;
    end
end