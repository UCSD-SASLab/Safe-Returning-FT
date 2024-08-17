close all; clear all; clc

% dataX = importdata('V_gamma=01_good.mat');
% g = importdata('g_good.mat');
% V_g=0.1_grid_-3_2-norm.mat
filename = 'V_g=0.1_5s_u_[pi2_0.3=d]_grid_-3_pos.mat';
dataX = load(filename, 'dataX').dataX;
g = load(filename, 'g').g;

radius = 0.06:0.01:2.5;
table = zeros(3,length(radius));

for i = 1:length(radius)
    level = radius(i);
    [levels, radius_set] = compute_level_set_radius(g, dataX, level,'separate');
    table(1,i) = level;
    table(2,i) = radius_set(1);
    table(3,i) = radius_set(2);
end
save('C:\Users\123\Desktop\Summer_2024\Zheng_play\online4D\Table_R_L_0.1_pos.mat',...
    "table");