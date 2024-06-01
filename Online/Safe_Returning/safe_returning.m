function [virt_x, JFlag, RpFlag, newStates, plot_jump]=safe_returning(trueQuad, virt_x,...
    level_set_X, level_set_Z, obsMap, senseRange,obs_index, num_obs,...
    goal,newStates, JFlag, cTEB, RpFlag, plot_jump)

delta_x = 0.025;

if JFlag == 1
    % find the level sets in the planner space
    obs_x_list = [-8 0 6]';
    [~, index] = min(abs(obs_x_list - trueQuad.x(1)));
    closest_x = obs_x_list(index);

    X_range = trueQuad.x(1) + level_set_X;
    Y_range = trueQuad.x(5) + level_set_X;
    level_set_Z = level_set_Z(1):(level_set_Z(end)-level_set_Z(1))/(length(level_set_X)-1):level_set_Z(end);
    Z_range = trueQuad.x(9) + level_set_Z';
    position_range = combvec(X_range', Y_range', Z_range');
    
    % find the obstalce free region
    bound = cTEB;
    distance = 0;
    while distance < bound
        [~, I] = min(vecnorm(position_range - goal));
        distance = compute_shortest_Dist(obsMap, senseRange, position_range(:,I), obs_index, num_obs);
        bound = cTEB;
        switch closest_x
            case -8
                if position_range(2,I)>3 || position_range(3,I)>5
                    bound = cTEB*sqrt(2);
                end
            case 0
                if position_range(2,I)<2
                    bound = cTEB*sqrt(2);
                end
            case 6
                if position_range(2,I)>5 || position_range(3,I)<-5
                    bound = cTEB*sqrt(2);
                end
        end
        if distance < bound
            position_range(:,I) = [];
        end
    end

    virt_x_old = virt_x;
    virt_x = position_range(:,I);

    RpFlag = 1;
    if plot_jump == 1
        x = [virt_x_old(1) virt_x(1)];
        y = [virt_x_old(2) virt_x(2)];
        z = [virt_x_old(3) virt_x(3)];
        plot3(x, y, z,'--og','linewidth', 2)
    end
else
    if isempty(newStates) || RpFlag == 1
        % Update next virtual state
        newStates = rrtNextState(virt_x, goal, obsMap.padded_obs, ...
            delta_x, [], false);
    end
    virt_x = newStates(1,:)';
    newStates(1,:) = [];
    RpFlag = 0;
end
JFlag = 0;
plot_jump = 0;
end



