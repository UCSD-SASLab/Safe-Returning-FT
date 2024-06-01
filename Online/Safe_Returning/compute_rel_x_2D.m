function [rel_x, temp] = compute_rel_x_2D(true, virt, gridX, dataX, Q)
rel_x = true - Q*virt;
tempX = eval_u(gridX, dataX, rel_x(1:4,:));
tempY = eval_u(gridX, dataX, rel_x(5:8,:));
temp = [tempX;tempY];
end