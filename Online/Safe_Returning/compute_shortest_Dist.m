function shortest_Dist = compute_shortest_Dist(obsMap, senseRange, position, obs_index, num_obs)

% Note that this algorithm only works for the current obstacles in the y-z plane

Distance = [];
if length(position)<4
    Q = zeros(10,3);
    Q(1,1) = 1;
    Q(5,2) = 1;
    Q(9,3) = 1;
    position = Q*position;
end
for j = 1:num_obs
    local_obs = [];
    for i = 1:size(obsMap.local_obs,1)
        if obs_index(j) ~=  obs_index(j+1)-1
            obstacle = squeeze(obsMap.local_obs(i,:,...
                obs_index(j):obs_index(j+1)-1));
        else
            obstacle = obsMap.local_obs(i,:,...
                obs_index(j):obs_index(j+1)-1)';
        end
        local_obs = [local_obs, obstacle];
    end
    points = unique(local_obs(2:3,:)','row');
    if ~isempty(local_obs)
[~, index] = min(position(1)-local_obs(1,:));
x_coordinate = local_obs(1, index);
        pgon = alphaShape(points);
        top_right = max(pgon.Points);
        bot_left = min(pgon.Points);
        top_left = [bot_left(1) top_right(2)];
        bot_right = [top_right(1) bot_left(2)];
    end

    if isempty(local_obs)
        Distance(j) = senseRange;
    elseif isequal((bot_left <= position([5 9])') + (position([5 9])'<= top_right), [2 2])
        Distance(j) = abs(position(1)- x_coordinate);
    elseif position(5) < bot_left(1) &&  (bot_left(2) <= position(9)) && (position(9) <= top_right(2))
        Distance(j) = norm(position([1 5 9])'- [x_coordinate bot_left(1) position(9)]);
    elseif position(5) > bot_right(1) &&  (bot_left(2) <= position(9)) && (position(9) <= top_right(2))
        Distance(j) = norm(position([1 5 9])'- [x_coordinate bot_right(1) position(9)]);
    elseif position(9) < bot_left(2) &&  (bot_left(1) <= position(5)) && (position(5) <= top_right(1))
        Distance(j) = norm(position([1 5 9])'- [x_coordinate position(5) bot_right(2)]);
    elseif position(9) > bot_left(2) &&  (bot_left(1) <= position(5)) && (position(5) <= top_right(1))
        Distance(j) = norm(position([1 5 9])'- [x_coordinate position(5) top_right(2)]);
    else
        [~, Distance(j)] = nearestNeighbor(pgon, position([5 9])');
        Distance(j) = sqrt(Distance(j)^2 + (position(1)- x_coordinate)^2);
    end

end
if isempty(Distance)
    shortest_Dist = senseRange;
else
    shortest_Dist = min(Distance);
end
if shortest_Dist > senseRange
    shortest_Dist = senseRange;
end
end