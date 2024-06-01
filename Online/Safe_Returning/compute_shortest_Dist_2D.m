function shortest_Dist = compute_shortest_Dist_2D(obs_disk, senseRange, position)
Distance = [];
for i = 1:obs_disk.num_obs
    x = obs_disk.center(i,1);
    y = obs_disk.center(i,2);
    distance = norm(position-[x;y]) - obs_disk.radius(i);
    Distance = [Distance distance];
end

shortest_Dist = min(Distance);
if shortest_Dist > senseRange
    shortest_Dist = senseRange;
end
end