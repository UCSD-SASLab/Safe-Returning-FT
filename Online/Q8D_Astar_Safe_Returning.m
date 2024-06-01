function [traj_time, comp_time]=Q8D_Astar_Safe_Returning()
addpath(genpath('.'))
clear all
close all
clc
% Problem setup
start = [1;1];
% start = [13;13];
goal = [16;16];
small = 0.1;
virt_v = 0.5;

% dt_offline = 0.1;
% delta_x = virt_v*dt_offline;
dt_online = 0.1;
% delta_x = virt_v*dt_online*2;

% Subsystems
XDims = 1:4;
YDims = 5:8;

data_filename = 'Q10D_Q3D_g0.1_20s_u_-0.5_grid_-5_0.05';

% matrix to compare position states (virt vs. true)
Q = zeros(8,2);
Q(1,1) = 1;
Q(5,2) = 1;

vis = true;

load(data_filename,'sD_X', 'dataX', 'derivX')

senseRange = 3;

%% Compute Smallest Invariant Set
[g1D_X, V_X] = proj(sD_X.grid, dataX, [0 1 1 1],'min');

[SCIS_X, TEB_X] = compute_level_set_radius(g1D_X, V_X, 0.01);
TEB = TEB_X;

%% Before Looping
uMode = 'min';
dMode = 'max'; % Not needed since we're not using worst-case control

% center = [4 5; 9 8; 10 3; 4 11; 12 12];
% r = [1.5 2 1.5 1 1]';
center = [6.5 7.5; 8 10];
r = [1.4 1]';
map = importdata('map_16.mat');
obs_disk = ObstacleMapAstar_2D(center, r);


% plot global obstacles
if vis
    rec_center =  [7 10];
%     r = [1.3]';
    width = 1.3;
    line_width = 4;
    Marker_size = 18;
    max_x = 16;
    max_y = 16;

    figure
    set(gcf,'unit','normalized','position',[0.2,0.1,0.64,0.8]);
        A = nsidedpoly(1000,'Center',center(1,:), 'Radius', r(1)+TEB);
        obs_1 = nsidedpoly(1000,'Center',center(1,:), 'Radius', r(1));
        hold on
    obs_2 = nsidedpoly(4,'Center',rec_center, 'SideLength', width*2);
    obs = union(obs_1, obs_2);
    B = nsidedpoly(4,'Center',rec_center, 'SideLength', (width+TEB)*2);
    C = union(A,B);
    plot(obs,'FaceColor', '#D41159','FaceAlpha',0.7,'EdgeColor','#D41159',...
        'LineWidth',line_width-3)
    plot(C,'FaceAlpha',0,'EdgeColor','#D41159','LineStyle','--', 'LineWidth',line_width)
    plot(goal(1), goal(2), "pentagram", 'MarkerFaceColor', 'r',...
        'MarkerEdgeColor','r', 'MarkerSize', Marker_size);
    % grid on
    axis([1 max_x+1 1 max_y+1])
    xticks(1:1:max_x+1)
    yticks(1:1:max_y+1)
end

% set initial states to zero
start_x = zeros(8,1);
start_x([1 5]) = start;
virt_x = start;

% Create real quadrotor system
rl_ui = [2 4];
trueQuad = Quad8D(start_x, sD_X.dynSys.uMin(rl_ui), sD_X.dynSys.uMax(rl_ui), ...
    sD_X.dynSys.dMin, sD_X.dynSys.dMax, 1:8);


% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block

newStates = [];
iter = 0;
sensed_new_1 = false;
true_x_last = trueQuad.x;


% writerObj = VideoWriter('Q8D_Q2D_SRF'); % Name it.
% writerObj.FrameRate = 20; % How many frames per second.
% open(writerObj);

tracker_traj = [];
planner_traj = [];
index_dstb = [];
JFlag = 0;
RpFlag = 1;
plot_jump = 0;
flag1 = 0;
count = 0;
trackErr = TEB;
%%
global_start = tic; % Time entire simulation
while norm(trueQuad.x([1 5]) - goal) > 0.3
    iter = iter + 1

    %     if iter == 1
    %         sensed_new_3 = obs_disk.sense_update(trueQuad.x([1 5]), senseRange, TEB);
    %     end
    %% Manual Disturbance
    %         if trueQuad.x(1) >=8 && trueQuad.x(5) >= 5.5 && flag1 == 0
    %              if trueQuad.x(1) >=8 && flag1 == 0
    %             trueQuad.x(1) = 6;
    %             trueQuad.x(5)= 5.3;
    %             flag1 = 1;
    %              end
    %     flag1 = rand;
    %     if flag1 <=0.05 && count<=1
    %         trueQuad.x(1) = trueQuad.x(1) - 0.2+0.4*rand;
    %         trueQuad.x(5) = trueQuad.x(5) - 0.2+0.4*rand;
    if trueQuad.x(1) >= 7 && count<=0
        trueQuad.x(1) = 5+0.5*rand;
        trueQuad.x(5) = 5.5+0.5*rand;
        trueQuad.x(1) = 6-rand;
        %         trueQuad.x(5) = 5+0.5*rand;
        if norm(trueQuad.x([1 5]) - center(1,:)')<=r(1)
            error('The disturbance is too large to handle!')
        end
        count = count+1;
        index_dstb = [index_dstb;iter];
    end
    [rel_x, ~] = compute_rel_x_2D(trueQuad.x, virt_x, sD_X.grid, dataX, Q);

    level_X_minmax = eval_u(g1D_X, V_X, rel_x(1));
    level_Y_minmax = eval_u(g1D_X, V_X, rel_x(5));
    if any([level_X_minmax==inf, level_Y_minmax==inf])
        error('The disturbance is too large to handle!')
    end
    %% Determine if the tracker is disturbed
    %     obs_sensed_previous = obsMap.seen_obs;
    if ~isequal(trueQuad.x, true_x_last)
        x = [trueQuad.x(1) true_x_last(1)];
        y = [trueQuad.x(5) true_x_last(5)];
        plot(x, y,'--ok', 'linewidth', 2)
        JFlag = 1;
        %         obs_sensed_previous = obsMap.seen_obs;
        sensed_new_1 = obs_disk.sense_update(trueQuad.x([1 5]), senseRange, trackErr);
    end

    %     local_start = tic;
    trueQuad.x([1 5])
    shortest_Dist = compute_shortest_Dist_2D(obs_disk, senseRange, trueQuad.x([1 5]));

    if shortest_Dist/2 > TEB
        radius_X = shortest_Dist;
        level = shortest_Dist/2;
        while radius_X >= shortest_Dist/2+small
            level = level - 0.01;
            [level_set_X, radius_X] = compute_level_set_radius(g1D_X, V_X, level);
        end
        cTEB = radius_X;
    else
        level_set_X = SCIS_X;
        cTEB = TEB;
    end

    sensed_new_2 = obs_disk.sense_update(trueQuad.x([1 5]), senseRange, cTEB);

    if sensed_new_1 || sensed_new_2
        RpFlag = 1;
    end

    if max([level_X_minmax, level_Y_minmax]) > small
        %        &&... shortest_Dist >= senseRange
        JFlag = 1;
        if isequal(trueQuad.x, true_x_last)
            plot_jump = 0;
        else
            plot_jump = 1;
        end
    elseif max([level_X_minmax, level_Y_minmax]) <= small
        if shortest_Dist >= senseRange
            JFlag = 1;
            %             plot_jump = 1;
        else
            JFlag = 0;
        end
    end

    [virt_x, JFlag, RpFlag, newStates, plot_jump] = Safe_Returning_Astar_2D(trueQuad,...
        virt_x, level_set_X, obs_disk, senseRange, goal,newStates,...
        JFlag, cTEB, RpFlag, plot_jump);

    planner_traj = [planner_traj virt_x];

    if ~isequal(trueQuad.x,true_x_last)
        sensed_new_1 = false;
        %         sensed_new_2 = false;
    end

    %% Tracking Block
    % 2. Determine which controller to use, find optimal control
    %get spatial gradients
    [rel_x, ~] = compute_rel_x_2D(trueQuad.x, virt_x, sD_X.grid, dataX, Q);
    pX = eval_u(sD_X.grid, derivX, rel_x(XDims));
    pY = eval_u(sD_X.grid, derivX, rel_x(YDims));

    % Find optimal control of relative system (no performance control)
    uX = sD_X.dynSys.optCtrl([], rel_x(XDims), pX, uMode);
    uY = sD_X.dynSys.optCtrl([], rel_x(YDims), pY, uMode);
    u = [uX uY];
    u = u(rl_ui);
    %     lookup_time = lookup_time + toc(local_start);
    %% True System Block
    % 1. add random disturbance to velocity within given bound
    %     d = sD_X.dynSys.dMin(1:2) + rand(2,1).*(sD_X.dynSys.dMax(1:2)...
    %         - sD_X.dynSys.dMin(1:2));
    d = [-0.1; -0.1]+ rand(2,1).*[0.2; 0.2];
    %     d(1) = d(1) + 0.1;

    % 2. update state of true vehicle
    trueQuad.updateState(u, dt_online, [], d);
    true_x_last = trueQuad.x;
    tracker_traj = [tracker_traj, trueQuad.x([1 5])];
    trueQuad.x([1 5])

    if norm(trueQuad.x([1 5]) - center(1,:)')<=r(1)
        error('Quadrotor ran into the obstacles!')
    end

    %% Virtual System Block
    trackErr = cTEB;
    % Visualize
    writerObj = [];
    showvideo = 0;
    if iter == 1
        hV =visualize_2D(obs_disk, trueQuad, virt_x, trackErr,...
            writerObj, iter, [], showvideo , RpFlag);
    else
        hV = visualize_2D(obs_disk, trueQuad, virt_x, trackErr,...
            writerObj, iter, hV, showvideo , RpFlag);
    end
end
comp_time = toc(global_start);
% comp_time = lookup_time;
traj_time = iter*dt_online
% if traj_time>=40
%     error('Quadrotor is too far from the goal!')
% end
% close(writerObj);
% filename = 'SRF_dist_3_obs_overlap_sense_1';
% save(filename, 'planner_traj', 'tracker_traj','traj_time');
end


