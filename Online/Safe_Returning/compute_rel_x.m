function [rel_x, temp] = compute_rel_x(true, virt, gridX, gridZ, dataX, dataZ, Q)
rel_x = true - Q*virt;
tempX = eval_u(gridX, dataX, rel_x(1:4,:));
tempY = eval_u(gridX, dataX, rel_x(5:8,:));
tempZ = eval_u(gridZ, dataZ, rel_x(9:10,:));
temp = [tempX;tempY;tempZ];
end