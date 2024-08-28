close all; clear all; clc
gamma = 0.8;
filename = sprintf('V_3D_g=%.1f_10s_u_[2_0.5=d]_grid_-3_pos.mat', gamma);
dataX = load(filename, 'dataX').dataX;
g = load(filename, 'g').g;

% radius = [0.01:0.01:1.6];
% table = zeros(3,160);
radius = min(dataX,[], 'all'):0.01:2.5;
table = zeros(3,length(radius));

for i = 1:length(radius)
    level = radius(i);
    [levels, radius_set] = compute_level_set_radius_ZG(g, dataX, level,'separate');
    table(1,i) = level;
    table(2,i) = radius_set(1);
    table(3,i) = radius_set(2);
end
table_name = sprintf('Table_3D_%.1f_grid_-3_pos.mat', gamma);
save(table_name, "table");