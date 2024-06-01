function [level_set, Radius] = compute_level_set_radius(grid, V, level)
index = find(V <= level);
ROES = cell2mat(grid.xs);
level_set = ROES(index);
Radius = max(abs(level_set));
end