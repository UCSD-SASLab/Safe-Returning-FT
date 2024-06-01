clear all
close all
%% Grid and cost
gNX = [121 121 21 31];
% gNX = [101 101 ceil(101/8) ceil(101/5)];
gNZ = gNX(1:2);

gMinX = [-5; -5; -35*pi/180; -5];
gMaxX = [ 5;  5;  35*pi/180;  5];
gMinZ = gMinX(1:2);
gMaxZ = gMaxX(1:2);

sD_X.grid = createGrid(gMinX, gMaxX, gNX);
sD_Z.grid = createGrid(gMinZ, gMaxZ, gNZ);

dataX0 = shapeCylinder(sD_X.grid,[],[0;0;0;0],0);
dataZ0 = shapeCylinder(sD_Z.grid,[],[0;0],0);
gamma = 0;
%% Parameters
gravity = 9.81;
uMin = [-0.5; -20/180*pi; -0.5; -20/180*pi; -0.5; 0];
uMax = [0.5; 20/180*pi; 0.5; 20/180*pi; 0.5; 1.5*gravity];
dMin = -[0.1; 0.1; 0.1]*0;
dMax = -dMin;
% dMax = [1/72*pi; 1/72*pi; 1/72*pi];
% dMin = -dMax;

uMode = 'min'; %10D trying to min
dMode = 'max'; %3D trying to max

sD_X.accuracy = 'veryHigh';
sD_Z.accuracy = 'veryHigh';
sD_X.uMode = uMode;
sD_X.dMode = dMode;
sD_Z.uMode = uMode;
sD_Z.dMode = dMode;
sD_Z.clf.gamma = gamma;
sD_X.clf.gamma = gamma;

figure(1)
clf
subplot(2,1,1)
hZ0 = surf(sD_Z.grid.xs{1},sD_Z.grid.xs{2},dataZ0);
xlabel('$z_r$','Interpreter','latex','FontSize',20)
ylabel('$v_z$','Interpreter','latex','FontSize',20)

subplot(2,1,2)
[g2DX,data2DX]=proj(sD_X.grid,dataX0,[0 0 1 1],[0 0]);
hX0 = surf(g2DX.xs{1},g2DX.xs{2},data2DX);
xlabel('$x_r$','Interpreter','latex','FontSize',20)
ylabel('$v_x$','Interpreter','latex','FontSize',20)

%% Dynamical systems and subsystems
Xdims = 1:4;
Zdims = 9:10;

sD_X.dynSys = Q10D_Q3D_Rel(zeros(10,1), uMin, uMax, dMin, dMax, Xdims);
sD_Z.dynSys = Q10D_Q3D_Rel(zeros(10,1), uMin, uMax, dMin, dMax, Zdims);


%% Z subsystem Solver parameters
HJIextraArgs.targetFunction = dataZ0;
HJIextraArgs.stopConverge = 1;
HJIextraArgs.convergeThreshold = 0.005;
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.valueFunction = 1;
HJIextraArgs.visualize.initialValueSet = 0;
HJIextraArgs.visualize.viewAngle = [30,45,30];
HJIextraArgs.visualize.figNum = 2; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.visualize.sliceLevel = 5;
HJIextraArgs.divergeThreshold = 8;
HJIextraArgs.stopDiverge = 1;
HJIextraArgs.target = dataZ0;
HJIextraArgs.keepLast = 1;
HJIextraArgs.ignoreBoundary = 1;

%% Run z subsystem
dt = 0.1;
tMax = 20;
tau = 0:dt:tMax;
[dataZ, tauZ] = HJIPDE_ZGsolve(dataZ0, tau, sD_Z, 'minCLF', HJIextraArgs);

%% X subsystems solver parameters
HJIextraArgs.targetFunction = dataX0;
HJIextraArgs.visualize.plotData.plotDims = [1 1 0 0];
HJIextraArgs.visualize.plotData.projpt = [0 0];
HJIextraArgs.visualize.sliceLevel = 3;
HJIextraArgs.visualize.figNum = 3;
HJIextraArgs.target = dataX0;


%% Run x subsystem
tic
[dataX, tauX] = HJIPDE_ZGsolve(dataX0, tau, sD_X, 'minCLF', HJIextraArgs);
runtime = toc ;
%% Find tracking error bound
TEB_Z = min(dataZ(:));
TEB_X = min(dataX(:));
TEB = max(TEB_Z,TEB_X);

%% Visualize
figure;
subplot(2,1,1)
hZ = surf(sD_Z.grid.xs{1}, sD_Z.grid.xs{2}, dataZ);
xlabel('$z_r$','Interpreter','latex','FontSize',20)
ylabel('$v_z$','Interpreter','latex','FontSize',20)

subplot(2,1,2)
[g2DX,data2DX]=proj(sD_X.grid, dataX,[0 0 1 1],[0 0]);
hX = surf(g2DX.xs{1},g2DX.xs{2},data2DX);
xlabel('$x_r$','Interpreter','latex','FontSize',20)
ylabel('$v_x$','Interpreter','latex','FontSize',20)

%% compute gradients (for controller)
derivX = computeGradients(sD_X.grid,dataX);
derivZ = computeGradients(sD_Z.grid,dataZ);

%% save
save(['Quad10D_g' num2str(gamma) '_dt0' num2str(dt*10) '_t' ...
num2str(tMax) '_' num2str(uMax([1 3 5])) '.mat'], 'TEB', 'TEB_X', 'TEB_Z', 'sD_X', ...
'sD_Z', 'dataX', 'dataZ', 'derivX', 'derivZ', 'gamma', '-v7.3')
%save(sprintf('%s_%f.mat', mfilename, now), 'sD_X', 'sD_Z', 'dataX', 'dataZ', ...
%  'tau', '-v7.3') 