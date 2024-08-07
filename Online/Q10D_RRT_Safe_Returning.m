function [traj_time, comp_time]=Q10D_RRT_Safe_Returning()
% addpath(genpath('.'))
clear all
close all
clc
% Problem setup
% start = [4.5; 0; 0];
% start = [-2.2; -3; 0];
start = [-14; 0; 0];
goal = [12; 0; 0];
small = 0.2;
virt_v = 0.5;

dt_online = 0.05;
delta_x = virt_v*dt_online;

% Subsystems
XDims = 1:4;
YDims = 5:8;
ZDims = 9:10;

% data_filename = 'Q10D_Q3D_g0.1_20s_u_-0.5_grid_-5_abs_0.1';
data_filename = 'Q10D_Q3D_g0.1_20s_u_-0.5_grid_-5_0.05';
obs_filename = 'obs.mat';

% matrix to compare position states (virt vs. true)
Q = zeros(10,3);
Q(1,1) = 1;
Q(5,2) = 1;
Q(9,3) = 1;

vis = true;
%% Load Data
load(data_filename, 'sD_X', 'dataX', 'derivX','sD_Z', 'dataZ', 'derivZ');
load(obs_filename,'obs')

senseRange = 3;

%% Compute Smallest Control Invariant Set
[g1D_X, V_X] = proj(sD_X.grid, dataX, [0 1 1 1],'min');
[g1D_Z, V_Z] = proj(sD_Z.grid, dataZ, [0 1],'min');

[SCIS_X, TEB_X] = compute_level_set_radius(g1D_X, V_X, 0.05);
[SCIS_Z, TEB_Z] = compute_level_set_radius(g1D_Z, V_Z, 0.05);
TEB = max(TEB_X, TEB_Z);
%% Before Looping
uMode = 'min';
dMode = 'max'; % Not needed since we're not using worst-case control

obsMap = ObstacleMapRRT(obs);

% plot global obstacles
if vis
    f = figure;
    f.Color = 'white';
    f.Position = [100 100 1280 720];

    obsMap.plotGlobal()
    f.Children.FontSize = 16;
    hold on
    plot3(goal(1), goal(2), goal(3), 'bo')

    xlabel('x', 'FontSize', 16)
    ylabel('y', 'FontSize', 16)
    zlabel('z', 'FontSize', 16)

    axis equal
    xlim([-15 15])
    %
    %   ylim([-10 10])
    %   zlim([-10 10])
    view([-10 10])
    box on
    grid on
end

% set initial states to zero
start_x = zeros(10,1);
start_x([1 5 9]) = start;
virt_x = start;
% virt_x = start+[0.5;0.5;0.5];
% virt_x = start+[2;2;2];

% Create real quadrotor system
rl_ui = [2 4 6];
trueQuad = Quad10D(start_x, sD_X.dynSys.uMin(rl_ui), sD_X.dynSys.uMax(rl_ui), ...
    sD_X.dynSys.dMin, sD_X.dynSys.dMax, 1:10);

% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];
%% Start loop! Tracking error bound block
newStates = [];
iter = 0;
sensed_new_1 = false;
true_x_last = trueQuad.x;

% writerObj = VideoWriter('SJ_cs_conv_2.avi'); % Name it.
% writerObj.FrameRate = 20; % How many frames per second.
% open(writerObj);

JFlag = 0;
RpFlag = 1;
plot_jump = 0;
flag1 = 0;
flag2 = 0;
flag3 = 0;
global_start = tic; % Time entire simulation

while norm(trueQuad.x([1 5 9]) - goal) > 0.5
    iter = iter + 1
    % update indices to compute the shortest distance to obs
    if isinf(obsMap.local_obs(1,1,1))
        obs_index = 1;
        num_obs = 0;
    end

    %% Manual Disturbance

    if trueQuad.x(1) >= -8 && trueQuad.x(1) <= -7.8 && flag1 == 0
        trueQuad.x(1) = -9 + 0.5*rand;
        trueQuad.x([5 9])= [2.5; trueQuad.x(9)-(1 + rand)];
        flag1 = 1;
    elseif trueQuad.x(1) >= 0 && trueQuad.x(1) <= 0.2 && flag2 == 0
        trueQuad.x(1) = -1 + 0.5*rand;
        trueQuad.x([5 9])= [3; trueQuad.x(9)-(1 + rand)];
        flag2 = 1;
    elseif trueQuad.x(1) >= 6 && trueQuad.x(1) <= 6.2 && flag3 == 0
        trueQuad.x(1) = 5 + 0.5*rand;
        trueQuad.x([5 9])= [trueQuad.x(5)-2-rand; trueQuad.x(9)+(2 + 0.5*rand)];
        flag3 = 1;
    end
    trueQuad.x([1 5 9]);
    %     dstb_flag = -1+2*rand;
    %     if abs(dstb_flag) <=2e-3
    %         temp = trueQuad.x([1 5 9]) +...
    %             [-1; -1; -1]+ rand(3,1).*2;
    %         flag = collision_check(temp, 1);
    %         while flag == 1
    %             temp = trueQuad.x([1 5 9]) +...
    %                 [-1; -1; -1]+ rand(3,1).*2;
    %             flag = collision_check(temp, 1);
    %         end
    %         trueQuad.x([1 5 9]) = temp;
    %         temp
    %     end
    [rel_x, ~] = compute_rel_x(trueQuad.x, virt_x, sD_X.grid, sD_Z.grid, dataX, dataZ, Q);

    level_X_minmax = eval_u(g1D_X, V_X, rel_x(1));
    level_Y_minmax = eval_u(g1D_X, V_X, rel_x(5));
    level_Z_minmax = eval_u(g1D_Z, V_Z, rel_x(9));
    if any([level_X_minmax==inf, level_Y_minmax==inf, level_Z_minmax==inf])
        error('The disturbance is too large to handle!')
    end
    %% Determine if the tracker is disturbed
    %     obs_sensed_previous = obsMap.seen_obs;
    if ~isequal(trueQuad.x, true_x_last)
        x = [trueQuad.x(1) true_x_last(1)];
        y = [trueQuad.x(5) true_x_last(5)];
        z = [trueQuad.x(9) true_x_last(9)];
        plot3(x, y, z,'--ok', 'linewidth', 2)
        %         JFlag = 1;
        %         obs_sensed_previous = obsMap.seen_obs;
        sensed_new_1 = obsMap.sense_update(trueQuad.x([1 5 9]), senseRange, trackErr);
    end

    % update indices to compute the shortest distance to obs
    [~, col] = find(squeeze(obsMap.local_obs(1,:,:))==inf,1);
    if isempty(col)
        col = size(obsMap.local_obs,3)+1;
    end
    if col~= 1 && col~= obs_index(end)
        obs_index = [obs_index col];
        num_obs = num_obs + 1;
    end

    shortest_Dist = compute_shortest_Dist(obsMap, senseRange, trueQuad.x, obs_index, num_obs);

    %% Find Safe Returning Region

    if shortest_Dist/2 > max(TEB_X, TEB_Z)
        radius_X = shortest_Dist;
        radius_Z = radius_X;
        level = shortest_Dist/2;
        while radius_X >= shortest_Dist/2
            level = level - 0.02;
            [level_set_X, radius_X] = compute_level_set_radius(g1D_X, V_X, level);
        end
        level = shortest_Dist/2;
        while radius_Z >= shortest_Dist/2
            level = level - 0.02;
            [level_set_Z, radius_Z] = compute_level_set_radius(g1D_Z, V_Z, level);
        end
        cTEB = max(radius_X, radius_Z);
    else
        level_set_X = SCIS_X;
        level_set_Z = SCIS_Z;
        cTEB = TEB;
    end

    %     obsMap.seen_obs = obs_sensed_previous;
    %% Augment Seen Obstacles with the new cTEB

    index_inf = (find(squeeze(obsMap.padded_obs(1,:,:))==inf,1)-1)/3;

    for i = 1:index_inf
        for j= 1:4
            x_list = unique(obsMap.local_obs(:,1,:));
            [~,index_x_orig] = min(abs(x_list - obsMap.padded_obs(j, 1, i)));
            if x_list(index_x_orig) >= obsMap.padded_obs(j, 1, i)
                obsMap.padded_obs(j, 1, i) = x_list(index_x_orig) - cTEB;
            else
                obsMap.padded_obs(j, 1, i) = x_list(index_x_orig) + cTEB;
            end

            y_list = unique(obsMap.local_obs(:,2,:));
            [~,index_y_orig] = min(abs(y_list - obsMap.padded_obs(j, 2, i)));
            if y_list(index_y_orig) >= obsMap.padded_obs(j, 2, i)
                obsMap.padded_obs(j, 2, i) = y_list(index_y_orig) - cTEB;
            else
                obsMap.padded_obs(j, 2, i) = y_list(index_y_orig) + cTEB;
            end
            z_list = unique(obsMap.local_obs(:,3,:));
            [~,index_z_orig] = min(abs(z_list - obsMap.padded_obs(j, 3, i)));
            if z_list(index_z_orig) >= obsMap.padded_obs(j, 3, i)
                obsMap.padded_obs(j, 3, i) = z_list(index_z_orig) - cTEB;
            else
                obsMap.padded_obs(j, 3, i) = z_list(index_z_orig) + cTEB;
            end
            if obsMap.padded_obs(j, 3, i)<=-6 &&...
                    abs(obsMap.padded_obs(j, 1, i)-6)<=1
                obsMap.padded_obs(j, 3, i) = -5 - cTEB;
            end
        end
    end

    %% Planning Block

    sensed_new_2 = obsMap.sense_update(trueQuad.x([1 5 9]), senseRange, cTEB);

    if sensed_new_1 || sensed_new_2
        RpFlag = 1;
    end

    if max([level_X_minmax, level_Y_minmax, level_Z_minmax]) > small
        JFlag = 1;
        if isequal(trueQuad.x, true_x_last)
            plot_jump = 0;
        else
            plot_jump = 1;
        end
    elseif max([level_X_minmax, level_Y_minmax, level_Z_minmax]) <= small
        if shortest_Dist >= senseRange
            JFlag = 1;
            %             plot_jump = 1;
        else
            JFlag = 0;
        end
    end

    [virt_x, JFlag, RpFlag, newStates, plot_jump]=safe_returning(trueQuad, virt_x,...
        level_set_X, level_set_Z, obsMap, senseRange,obs_index, num_obs,...
        goal,newStates, JFlag, cTEB, RpFlag, plot_jump);

        if ~isequal(trueQuad.x,true_x_last)
            sensed_new_1 = false;
%             sensed_new_2 = false;
        end
    if ~isempty(newStates)
        if any(newStates(:,2)<-10)|| any(newStates(:,2)>11) ||...
                any(newStates(:,3)<-10) || any(newStates(:,3)>10)
            error('RRT gives a path outside of domain!')
        end
    end
    %% Tracking Block
    % 2. Determine which controller to use, find optimal control
    %get spatial gradients
    [rel_x, ~] = compute_rel_x(trueQuad.x, virt_x, sD_X.grid, sD_Z.grid, dataX, dataZ, Q);
    pX = eval_u(sD_X.grid, derivX, rel_x(XDims));
    pY = eval_u(sD_X.grid, derivX, rel_x(YDims));
    pZ = eval_u(sD_Z.grid, derivZ, rel_x(ZDims));

    % Find optimal control of relative system (no performance control)
    uX = sD_X.dynSys.optCtrl([], rel_x(XDims), pX, uMode);
    uY = sD_X.dynSys.optCtrl([], rel_x(YDims), pY, uMode);
    uZ = sD_Z.dynSys.optCtrl([], rel_x(ZDims), pZ, uMode);

    u = [uX uY uZ];
    u = u(rl_ui);

    %% True System Block
    % 1. add random disturbance to velocity within given bound
    %     d = sD_X.dynSys.dMin + rand(3,1).*(sD_X.dynSys.dMax - sD_X.dynSys.dMin);
    d = [-0.1; -0.1; -0.1]+ rand(3,1).*[0.2; 0.2; 0.2];
    %         d(1) = d(1) + 0.1;

    % 2. update state of true vehicle
    trueQuad.updateState(u, dt_online, [], d);
    collision_check(trueQuad.x([1 5 9]), 0);
    true_x_last = trueQuad.x;

    %% Virtual System Block
    trackErr = cTEB;
    % Visualize

    if iter == 1
        [hV, boxMap]=visualize(obsMap, trueQuad, virt_x, trackErr, [], iter,[],...
            [], 0, RpFlag);
    else
        [hV, boxMap]=visualize(obsMap, trueQuad, virt_x, trackErr, [], iter,boxMap,...
            hV, 0, RpFlag);
    end
end
comp_time = toc(global_start);
traj_time = iter*dt_online;
% close(writerObj);

end
function flag = collision_check(state, mode)
%if (-11<=trueQuad.x(1) && trueQuad.x(1)<=-10.8 &&...
%       trueQuad.x(5)>=-3 && trueQuad.x(5)<=7) ||...
flag = 0;
if       (-8.1<=state(1) && state(1)<=-7.9 &&...
        state(2)<=3 && state(3)<=5) ||...
        (-0.1<=state(1) && state(1)<=0.1 && state(2)>=2 &&state(2)<=10) ||...
        (5.9<=state(1) && state(1)<=6.1 && state(2)<=5 &&...
        state(3)>=-5)
    if mode == 1
        flag = 1;
    elseif mode == 0
        error('Quadrotor ran into the obstacles!')
    end
end
end

