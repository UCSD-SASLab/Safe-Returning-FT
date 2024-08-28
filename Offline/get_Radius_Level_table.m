close all; clear all; clc

% dataX = importdata('V_gamma=01_good.mat');
% g = importdata('g_good.mat');
% V_g=0.1_grid_-3_2-norm.mat
gamma = 0.1;
% filename = sprintf('V_3D_g=%.1f_10s_u_[2_0.5=d]_grid_-3_pos.mat', gamma);
filename = sprintf('V_4D_g=%.1f_5s_u_[pi_0.3=d]_grid_-3_Q_pos.mat', gamma);
dataX = load(filename, 'dataX').dataX;
g = load(filename, 'g').g;

radius = min(dataX,[], 'all'):0.01:2.5;
table = zeros(3,length(radius));

for i = 1:length(radius)
    level = radius(i);
    [levels, radius_set] = compute_level_set_radius_ZG(g, dataX, level,'separate');
    table(1,i) = level;
    table(2,i) = radius_set(1);
    table(3,i) = radius_set(2);
end
table_name = sprintf('Table_4D_%.1f_grid_-3_Q_pos.mat', gamma);
save(table_name, "table");
% save('C:\Users\123\Desktop\Summer_2024\Zheng_play\online4D\Table_R_L_0.1_pos.mat',...
%     "table");