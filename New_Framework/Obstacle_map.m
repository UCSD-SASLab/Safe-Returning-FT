close all; clear all; clc 

%% maximum map range
max_x = 20; 
max_y = 20;
ground=3*(ones(max_x,max_y)); % 3 means unsensed map. turn to 2 once sensed

%% Target 
Target_x = 18;
Target_y = 18;
ground(Target_x,Target_y) = 0;

%% Obstacles 
   %  first define vertices of the Obs.
xv1 = [4,6,6,4];
yv1 = [4,4,6,6];

xv2 = [8,13,13,8];
yv2 = [9,9,12,12];

xv3 = [10,13,13,10];
yv3 = [3,3,4,4];
    % transform obs vertices to obs grid
obs1_grid = Vert2grid(xv1,yv1);
obs2_grid = Vert2grid(xv2,yv2);
obs3_grid = Vert2grid(xv3,yv3);
obs_grid = [obs1_grid,obs2_grid,obs3_grid];

Obs_x = obs_grid(1,:);
Obs_y = obs_grid(2,:);
num_obs = length(Obs_x);

for i = 1 : num_obs
    ground(Obs_x(i),Obs_y(i)) = -3; %-3 means unsensed obs grid, turns to 
                                    % -2 once sensed
end

Map.xMax = max_x;
Map.yMax = max_y;
Map.xTarget = Target_x;
Map.yTarget = Target_y;
Map.ground = ground;
Map.obs.vx1 = [xv1;yv1];
Map.obs.vx2 = [xv2;yv2];
Map.obs.vx3 = [xv3;yv3];
Map.obs.grid1 = obs1_grid;
Map.obs.grid2 = obs2_grid;
Map.obs.grid3 = obs3_grid;
Map.obs.grid = obs_grid;
save('Map.mat','Map');