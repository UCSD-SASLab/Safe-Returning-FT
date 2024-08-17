function [level_set, Radius] = compute_level_set_radius_ZG(grid, V, level,method)
    % find the TEB and corresponding radius
    % method: 
    %       uniform: find radius for each dimsion and output the max of
    %       them
    %       separate: find radius for each and output each
if nargin < 4 
    method = 'uniform';
end

dim = grid.dim;
index = find(V <= level);
stateSpace = cell2mat(grid.xs);
level_set = stateSpace(index);
Radius = cell(dim,1);

switch method
    case 'separate'
        Radius = zeros(dim,1);
        level_set = cell(dim,1);
        for i = 1 : dim
            level_set{i} = grid.xs{i}(index);
            Radius(i) = max(abs(level_set{i}));
        end
    case 'uniform'
        level_set = stateSpace(index);
        Radius = max(abs(level_set));
end
end