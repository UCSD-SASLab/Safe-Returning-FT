function [virt_x, JFlag, RpFlag, newStates, plot_jump] = Safe_Correction_Astar_2D(trueQuad,...
    virt_x, level_set_X, obs_disk, senseRange, goal,newStates,...
    JFlag, cTEB, RpFlag, plot_jump)
%% Path Planning Block
delta_x = 0.025;

if JFlag == 1
    X_range = trueQuad.x(1) + level_set_X;
    Y_range = trueQuad.x(5) + level_set_X;
    position_range = combvec(X_range', Y_range');
        Exclude_index = [];
    Valid_dist = [];
    for i = 1:length(position_range)
        distance = compute_shortest_Dist_2D(obs_disk, senseRange, position_range(:,i));
        if distance < cTEB
           Exclude_index = [Exclude_index i];
        else
            Valid_dist = [Valid_dist distance];
        end
    end
     position_range(:,Exclude_index) = [];
     [~, I] = min(vecnorm(position_range - goal));
%     bound = cTEB;
%     distance = 0;
%     while distance < bound
%         [~, I] = min(vecnorm(position_range - goal));
%         distance = compute_shortest_Dist_2D(obs_disk, senseRange, position_range(:,I));
%         bound = cTEB;
%         if distance < bound
%             position_range(:,I) = [];
%         end
%     end
%    if cTEB<=0.5
%        keyboard
%    end
    virt_x_old = virt_x;
    virt_x = position_range(:,I);
    compute_shortest_Dist_2D(obs_disk, senseRange, virt_x)
    RpFlag = 1;
    if plot_jump == 1
        x = [virt_x_old(1) virt_x(1)];
        y = [virt_x_old(2) virt_x(2)];
        plot(x, y, '--og','linewidth', 2)
    end

else
    if isempty(newStates) || RpFlag == 1
        % Update next virtual state
        newStates = AStar_path(virt_x(1), virt_x(2), max(obs_disk.dr));
    end
    %     if isempty(newStates)
    %         keyboard
    %     end
    virt_x = newStates(1,:)';
    newStates(1,:) = [];
    RpFlag = 0;
end
JFlag = 0;
plot_jump = 0;
end



