function [virt_x, JFlag, RpFlag, newStates, plot_jump, plot_replan] = Safe_Returning_Astar_2D_poly...
    (trueQuad, virt_x, level_set, obs_map, senseRange, goal,newStates,...
    JFlag, cTEB, RpFlag, plot_jump, plot_replan)
%% Path Planning Block
delta_x = 0.05;

if JFlag == 1
    X_range = trueQuad.x(1) + level_set(:,1);
    Y_range = trueQuad.x(2) + level_set(:,2);
    position_range = [X_range'; Y_range'];
        Exclude_index = [];
    Valid_dist = [];
    for i = 1:length(position_range)
        distance = compute_shortest_Dist_2D_poly(obs_map, senseRange, position_range(:,i));
        if distance < max(cTEB)
           Exclude_index = [Exclude_index i];
        else
            Valid_dist = [Valid_dist distance];
        end
    end
     position_range(:,Exclude_index) = [];
     [~, I] = min(vecnorm(position_range - goal));
    virt_x_old = virt_x;
    virt_x = position_range(:,I);
    RpFlag = 1;
    if plot_jump == 1
        x = [virt_x_old(1) virt_x(1)];
        y = [virt_x_old(2) virt_x(2)];
        plot(x, y, '--og','linewidth', 2)
    end

else
    if isempty(newStates) || RpFlag == 1
        % Update next virtual state
        newStates = AStar_path_poly(virt_x(1), virt_x(2), obs_map, cTEB);
        plot_replan = 1;
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



