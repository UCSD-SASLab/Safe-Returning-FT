close all
clear 
clc


%% Grid
grid_min = [-5; -5; -pi]; % Lower corner of computation domain
grid_max = [5; 5; pi];    % Upper corner of computation domain
N = [121; 121; 91];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% time vector
t0 = 0;
tMax = 10;
dt = 0.1;
tau = t0:dt:tMax;

%% problem parameters
uMode = 'min';
dMode = 'max';



%% Dynamics
params.w = pi/2; %
params.v = [-2, 2]; % 
params.d = [0.5; 0.5; 0]; % 
 
x0 = [0;0;0];
dCar = Dubins3D(x0, params.w , params.v, params.d);

gamma = 0;

%% Pack problem parameteres
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
schemeData.clf.gamma = gamma;

%% Compute the CLVF using algorithm 1. 
% First find the smallest value for the value function with gamma = 0
% Test with different cost functions:
% 1: l(x) = ||x||_2, 
% 2: l(x) = ||x||_infty
% 3: l(x) = sqrt(x'Qx)
Q = [10,0,0;0,10,0;0,0,0.1];
data0 = QuadCost(g,Q);

% data0 = shapeCylinder(g, 3, [0; 0; 0], 0) ;
% data_filename = 'V_g=0_5s_u_[1_0.1=d]_grid_-3_pos.mat';
% data_filename ='V_3D_g=0_10s_u_[2_0.5=d]_grid_-5_pos.mat';
% data0 = load(data_filename, 'dataX').dataX - load(data_filename, 'TEB').TEB - 0.1;
min(data0, [], 'all')

cost_type = 'Q_pos';
cost = data0;

[dataX,tau1] = ComputeHJ(data0,cost,tau,schemeData);
TEB = min(dataX,[],'all');

% save('V_g=0.1_grid_-3_abs.mat')
% save('g_fine.mat','g')
% filename = sprintf('V_3D_g%.1f_5s_u_[1_0.1=d]_grid_-3_%s.mat', gamma, cost_type);
filename = sprintf('V_3D_g=%.1f_10s_u_[-2_0.5=d]_grid_-5_%s.mat', gamma, cost_type);
save(filename)
%% Compute CLVF with gamma \neq 0
% The cost function l(x) = data0 - c
% but we can warm start with IC = V_0 - c 

% data0_2 = data0-2;
% [V_2,tau2] = ComputeHJ(data0_2,tau,schemeData);
% % save('V_gamma=03_fine.mat','V_2')



%%
function [data,tau] = ComputeHJ(data0,cost,tau0,schemeData)
% data0: initial value function
% cost: cost function
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.valueFunction = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.deleteLastPlot = true; 
HJIextraArgs.convergeThreshold = 0.005;
HJIextraArgs.stopConverge = 1;
HJIextraArgs.divergeThreshold = 8;
HJIextraArgs.ignoreBoundary = 1;
HJIextraArgs.keepLast = 1;
HJIextraArgs.makeVideo = 0;
HJIextraArgs.targetFunction = cost;
HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; 
HJIextraArgs.visualize.plotData.projpt = {'min'}; 
HJIextraArgs.visualize.viewAngle = [30,30];

[data, tau, ~] = ...
  HJIPDE_ZGsolve(data0, tau0, schemeData, 'minCLF', HJIextraArgs);

end