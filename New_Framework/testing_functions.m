close all; clear all; clc

Map = importdata('Map.mat');

x = 7;
y = 7; 
senseRange = 3;
TEB = [1.4,0.6];
dim = 2;

[SenseMap,old_map] = SenseEnv(x,y,senseRange,Map,TEB,dim);

newStates = AStar_path(x, y, SenseMap.aug_obs.grid);

obs_1 = polyshape(Map.obs.vx1(1,:),Map.obs.vx1(2,:));
obs_2 = polyshape(Map.obs.vx2(1,:),Map.obs.vx2(2,:));
obs_3 = polyshape(Map.obs.vx3(1,:),Map.obs.vx3(2,:));
obs = union(obs_1, union(obs_2,obs_3));

[aug_xv1,aug_yv1] = AugObsVx(Map.obs.vx1(1,:),Map.obs.vx1(2,:),TEB);
[aug_xv2,aug_yv2] = AugObsVx(Map.obs.vx2(1,:),Map.obs.vx2(2,:),TEB);
[aug_xv3,aug_yv3] = AugObsVx(Map.obs.vx3(1,:),Map.obs.vx3(2,:),TEB);

aug_obs1 = polyshape(aug_xv1,aug_yv1);
aug_obs2 = polyshape(aug_xv2,aug_yv2);
aug_obs3 = polyshape(aug_xv3,aug_yv3);

obs_aug = union(aug_obs1, union(aug_obs2,aug_obs3));

%%
figure
set(gcf,'unit','normalized','position',[0.2,0.1,0.64,0.8]);
hold on
plot(obs,'FaceColor', '#D41159','FaceAlpha',0.7,'EdgeColor','#D41159',...
    'LineWidth',1)
plot(obs_aug,'FaceAlpha',0,'EdgeColor','#D41159','LineStyle','--', 'LineWidth',1)
plot(Map.xTarget, Map.yTarget, "pentagram", 'MarkerFaceColor', 'r',...
    'MarkerEdgeColor','r', 'MarkerSize', 15);
plot(newStates(:,1),newStates(:,2),'.')
grid on
axis([0 Map.xMax+2 0 Map.yMax+2])
xticks(1:1:Map.xMax+2)
yticks(1:1:Map.yMax+2)