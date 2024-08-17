clear all
close all
clc
% Problem setup
start = [3;3];
% start = [13;13];
goal = [16;16];
small = 0.1;
virt_v = 0.5;

% dt_offline = 0.1;
% delta_x = virt_v*dt_offline;
dt_online = 0.05;
% delta_x = virt_v*dt_online*2;

% Subsystems
XDims = 1;
YDims = 2;


% matrix to compare position states (virt vs. true)
Q = zeros(4,2);
Q(1,1) = 1;
Q(2,2) = 1;

vis = true;

% dataX = importdata('V_gamma=01_good.mat');
% g = importdata('g_good.mat');
% Table_R_L = importdata('Table_R_L.mat');
% V_g=0.1_grid_-3_2-norm.mat
filename = 'V_g=0.1_5s_u_[pi2_0.3=d]_grid_-3_pos.mat';
dataX = load(filename, 'dataX').dataX;
g = load(filename, 'g').g;
TEB = load(filename, 'TEB').TEB;
Table_R_L = importdata('Table_R_L_0.1_pos.mat');
deriv = computeGradients(g,dataX);
senseRange = 3;

%% Compute Smallest Invariant Set
% [g1D_X, V_X] = proj(g, dataX, [0 1 1 1],'min');

[SCIS, TEB] = compute_level_set_radius_ZG(g, dataX, TEB+0.02,'separate');
TEB_X = TEB(1);
SCIS_X = SCIS{1};

TEB_Y = TEB(2);
SCIS_Y = SCIS{2};

%% Before Looping
uMode = 'min';
dMode = 'max'; % Not needed since we're not using worst-case control

% center = [4 5; 9 8; 10 3; 4 11; 12 12];
% r = [1.5 2 1.5 1 1]';
xv1 = [5,7,7,5];
yv1 = [4,4,6,6];

xv2 = [8,13,13,8];
yv2 = [9,9,12,12];

xv3 = [10,13,13,10];
yv3 = [3,3,4,4];

xv = [xv1; xv2; xv3];
yv = [yv1; yv2; yv3];

aug_x = [-TEB_X, TEB_X, TEB_X, -TEB_X];
aug_y = [-TEB_Y, -TEB_Y, TEB_Y, TEB_Y];
xv_aug = xv + aug_x;
yv_aug = yv + aug_y;

xv1_aug = [5-TEB_X,7+TEB_X,7+TEB_X,5-TEB_X];
yv1_aug = [4-TEB_Y,4-TEB_Y,6+TEB_Y,6+TEB_Y];

xv2_aug = [8-TEB_X,13+TEB_X,13+TEB_X,8-TEB_X];
yv2_aug = [9-TEB_Y,9-TEB_Y,12+TEB_Y,12+TEB_Y];

xv3_aug = [10-TEB_X,13+TEB_X,13+TEB_X,10-TEB_X];
yv3_aug = [3-TEB_Y,3-TEB_Y,4+TEB_Y,4+TEB_Y];

% xv_aug(1,:) == xv1_aug
% xv_aug(2,:) == xv2_aug
% xv_aug(3,:) == xv3_aug
% yv_aug(1,:) == yv1_aug
% yv_aug(2,:) == yv2_aug
% yv_aug(3,:) == yv3_aug
map = importdata('map_16.mat');
opt_traj = map.opttraj;


obs_map = ObstacleMapAstar_2D_rectangle(xv, yv);

% reshape(mat2cell(xv,ones(1,size(xv,1))), 1, [])
% polyshape(reshape(mat2cell(xv,ones(1,size(xv,1))), 1, []),...
%     reshape(mat2cell(yv,ones(1,size(yv,1))), 1, []))
% plot global obstacles
if vis
%     r = [1.3]';
    line_width = 4;
    Marker_size = 18;
    max_x = 16;
    max_y = 16;

    main_fig = figure;
    % set(gcf,'unit','normalized','position',[0.2,0.1,0.64,0.8]);

    obs_map.plotGlobal()
    hold on
    % obs_map.plotPadded(TEB_X, TEB_Y)
    % obs_1 = polyshape(xv1,yv1);
    % obs_2 = polyshape(xv2,yv2);
    % obs_3 = polyshape(xv3,yv3);
    % A = polyshape(xv1_aug,yv1_aug);
    % B = polyshape(xv2_aug,yv2_aug);
    % C = polyshape(xv3_aug,yv3_aug);
    % 
    % hold on
    % obs = union(obs_1, union(obs_2,obs_3));
    % D = union(A,union(B,C));
    % plot(obs,'FaceColor', '#D41159','FaceAlpha',0.7,'EdgeColor','#D41159',...
    %     'LineWidth',line_width-3)
    % plot(D,'FaceAlpha',0,'EdgeColor','#D41159','LineStyle','--', 'LineWidth',line_width)
    plot(goal(1), goal(2), "pentagram", 'MarkerFaceColor', 'r',...
        'MarkerEdgeColor','r', 'MarkerSize', Marker_size);
    % plot(opt_traj(:,1),opt_traj(:,2))
    % grid on
    axis([0 max_x+1 0 max_y+1])
    xticks(1:1:max_x+1)
    yticks(1:1:max_y+1)
end

% set initial states to zero
start_x = zeros(4,1);
start_x([1 2]) = start;
virt_x = start;

% Create real quadrotor system
% rl_ui = [2 4];
trueQuad = Plane4D(start_x, pi/2, [-2,2], [0.3,0.3]);

u_cell = trueQuad.optCtrl([], [], deriv, uMode);
u1 = u_cell{1};
u2 = u_cell{2};
% % define when to switch from safety control to performance control
% small = 1;
% safetyBound = bubble - small.*[gs{1}.dx(1); gs(2).dx(1); gs(3).dx(1)];

%% Start loop! Tracking error bound block
path_handle = [];
newStates = [];
iter = 0;
sensed_new_1 = false;
true_x_last = trueQuad.x;


writerObj = VideoWriter('Q4D_Q2D_SRF_g=0.1_pos', 'MPEG-4'); % Name it.
writerObj.FrameRate = 20; % How many frames per second.
open(writerObj);

tracker_traj = [];
planner_traj = [];
value_traj = [];
index_dstb = [];
u_signal = [];
JFlag = 0;
RpFlag = 1;
plot_jump = 0;
plot_replan = 0;
flag1 = 0;
count = 1;
trackErr = [TEB_X,TEB_Y];
%%
global_start = tic; % Time entire simulation
while norm(trueQuad.x([1 2]) - goal) > 0.3
    iter = iter + 1

    %     if iter == 1
    %         sensed_new_3 = obs_map.sense_update(trueQuad.x([1 5]), senseRange, TEB);
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
    % if trueQuad.x(1) >= 7 && count<=0
    %     trueQuad.x(1) = 4+0.5*rand;
    %     trueQuad.x(2) = 5+0.5*rand;
    %     [d1,~,~] = p_poly_dist(trueQuad.x(1), trueQuad.x(2), xv1, yv1);
    %     [d2,~,~] = p_poly_dist(trueQuad.x(1), trueQuad.x(2), xv2, yv2);
    %     d = min(d1,d2);
    %     % trueQuad.x(1) = 6-rand;
    %     %         trueQuad.x(5) = 5+0.5*rand;
    %     if d<=0
    %         error('The disturbance causes crash')
    %     end
    %     count = count+1;
    %     index_dstb = [index_dstb;iter];
    % end
    % [rel_x, ~] = compute_rel_x_2D(trueQuad.x, virt_x, sD_X.grid, dataX, Q);
    rel_x =  trueQuad.x - Q*virt_x;
    %% 
    level_rel = eval_u(g, dataX, rel_x);
    if level_rel == inf
        error('The disturbance causes rel state leave ROES')
    end
    %% Determine if the tracker is disturbed
    %     obs_sensed_previous = obsMap.seen_obs;
    if ~isequal(trueQuad.x, true_x_last)
        x = [trueQuad.x(1) true_x_last(1)];
        y = [trueQuad.x(2) true_x_last(2)];
        plot(x, y,'--ok', 'linewidth', 2)
        JFlag = 1;
        %         obs_sensed_previous = obsMap.seen_obs;
        sensed_new_1 = obs_map.sense_update(trueQuad.x([1 2]), senseRange, cTEB(1), cTEB(2));
    end

    %     local_start = tic;
    trueQuad.x([1 2]);
    shortest_Dist = compute_shortest_Dist_2D_poly(obs_map, senseRange, trueQuad.x([1 2]));
    % d_list = nan(1, size(xv,1));
    % for i = 1:size(xv,1)
    %     [d, ~] = p_poly_dist(trueQuad.x(1), trueQuad.x(2), xv(i,:), yv(i,:));
    %     d_list(i) = d;
    % end
    % [shortest_Dist,I] = min(d_list);
    % % xd = abs(trueQuad.x(1) - xp(I));
    % % yd = abs(trueQuad.x(2) - yp(I));
    % if shortest_Dist> senseRange
    %     shortest_Dist = senseRange;
    % end
    % shortest_Dist = compute_shortest_Dist_2D(obs_map, senseRange, trueQuad.x([1 2]));

    if shortest_Dist/2 > max(TEB_X,TEB_Y)
        radius = shortest_Dist;

        X_R_ind = max(find(Table_R_L(2,:) <= shortest_Dist/2+small));
        Y_R_ind = max(find(Table_R_L(3,:) <= shortest_Dist/2+small));
        R_ind = min(X_R_ind,Y_R_ind);
        radius_X = Table_R_L(2,R_ind);
        radius_Y = Table_R_L(3,R_ind);
        level = Table_R_L(1,R_ind);
        [levels, ~] = compute_level_set_radius_ZG(g, dataX, level,'separate');

        % while radius >= shortest_Dist/2+small
        %     level = level - 0.01;
        %     [levels, radius_set] = compute_level_set_radius(g, dataX, level,'separate');
        %     radius = max(radius_set(1:2));
        % end
        level_set = unique([levels{1}, levels{2}], 'rows', 'first');
        level_set_X = level_set(:,1);
        level_set_Y = level_set(:,2);
        % a=unique([level_set_X, level_set_Y], 'rows', 'first');
        % radius_X = radius_set(1);
        % radius_Y = radius_set(2);
        cTEB = [radius_X,radius_Y];
        level_set=[level_set_X,level_set_Y];
    else
        level_set_X = SCIS_X;
        level_set_Y = SCIS_Y;
        cTEB = [TEB_X,TEB_Y];
        level_set=[level_set_X,level_set_Y];
        level = 0.01;
    end

    sensed_new_2 = obs_map.sense_update(trueQuad.x([1 2]), senseRange, cTEB(1), cTEB(2));

    if sensed_new_1 || sensed_new_2
        RpFlag = 1;
    end
      
    if level_rel > small+0.3
        %        &&... shortest_Dist >= senseRange
        JFlag = 1;
        if isequal(trueQuad.x, true_x_last)
            plot_jump = 0;
        else
            plot_jump = 1;
        end
    elseif level_rel <= small+0.3
        if shortest_Dist >= senseRange
            JFlag = 1;
            %             plot_jump = 1;
        else
            JFlag = 0;
            % plot_replan = 1;
        end
    end
% tic
[virt_x, JFlag, RpFlag, newStates, plot_jump, plot_replan] = Safe_Returning_Astar_2D_poly(trueQuad,...
    virt_x, level_set, obs_map, senseRange, goal,newStates,...
    JFlag, cTEB, RpFlag, plot_jump, plot_replan);
% toc
% tic
    if length(newStates)>1 && isequal(plot_replan, 1)
        if ~isempty(path_handle)
            delete(path_fig)
        end
        figure(main_fig)
        path_fig = plot(newStates(:,1),newStates(:,2));
        plot_replan = 0;
    end
    planner_traj = [planner_traj virt_x];

    if ~isequal(trueQuad.x,true_x_last)
        sensed_new_1 = false;
        %         sensed_new_2 = false;
    end

    %% Tracking Block
    % 2. Determine which controller to use, find optimal control
    %get spatial gradients
    % [rel_x, ~] = compute_rel_x_2D(trueQuad.x, virt_x, sD_X.grid, dataX, Q);
    rel_x =  trueQuad.x - Q*virt_x;

    pX = eval_u(g, deriv, rel_x);
    uX = trueQuad.optCtrl([], rel_x, pX, uMode);
    uX = cell2mat(uX');
    u_signal = [u_signal; uX];
    % Find optimal control of relative system (no performance control)
    % if isequal(rel_x, zeros(4,1))
    %     u= [0, 0];
    % else
        u_x = eval_u(g,u1,rel_x);
        u_y = eval_u(g,u2,rel_x);
        u = [u_x,u_y];
    % if ~isequal(u, uX)
    %     keyboard
    % end

    %% True System Block
    % 1. add random disturbance to velocity within given bound
        % d = sD_X.dynSys.dMin(1:2) + rand(2,1).*(sD_X.dynSys.dMax(1:2)...
            % - sD_X.dynSys.dMin(1:2));
    % d = [-0.1; -0.1]+ rand(2,1).*[0.2; 0.2];
    d = [0;0];
    %     d(1) = d(1) + 0.1;

    % 2. update state of true vehicle
    trueQuad.updateState(uX, dt_online, [], d);
    trueQuad.x([1 2])
    true_x_last = trueQuad.x;
    tracker_traj = [tracker_traj; trueQuad.x([1 2])];
    % current_state = trueQuad.x([1 2])
    value = eval_u(g,dataX,trueQuad.x - Q*virt_x)
    value_traj = [value_traj; value];
    % if norm(trueQuad.x([1 2]) - center(1,:)')<=r(1)
    %     error('Quadrotor ran into the obstacles!')
    % end

    %% Virtual System Block
    % Visualize
    showvideo = 1;
    % tic
    if iter == 1
        [hV, box_map] = visualize_2D_Plane4D(obs_map, [], trueQuad, virt_x, cTEB,...
            writerObj, iter, [], showvideo, RpFlag);
    else
        [hV, box_map] = visualize_2D_Plane4D(obs_map, box_map, trueQuad, virt_x, cTEB,...
            writerObj, iter, hV, showvideo, RpFlag);
    end
    toc
    % if iter == 1
    %     hLS = visLevelSet(trueQuad, g,dataX ,writerObj, [],...
    %         showvideo , JFlag);
    % else
    %     hLS = visLevelSet(trueQuad, g,dataX ,writerObj, hLS,...
    %         showvideo , JFlag);
    % end
end
comp_time = toc(global_start);
% comp_time = lookup_time;
traj_time = iter*dt_online
% save('fail_3','tracker_traj','value_traj', 'u_signal')
% if traj_time>=40
%     error('Quadrotor is too far from the goal!')
% end
close(writerObj);
% filename = 'SRF_dist_3_obs_overlap_sense_1';
% save(filename, 'planner_traj', 'tracker_traj','traj_time');
