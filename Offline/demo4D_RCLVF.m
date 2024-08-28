close all
clear
clc


%% Grid
grid_min = [-3; -3; -pi; -3]; % Lower corner of computation domain
grid_max = [3; 3; pi; 3];    % Upper corner of computation domain
N = [75; 75; 35; 25];         % Number of grid points per dimension
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
params.w = pi/2; % acceleration of the Dubins car
params.a = [-2, 2]; %
params.d = [0.3; 0.3]; %

x0 = [0;0;0;0];
dCar = Plane4D(x0, params.w , params.a, params.d);

gamma = 1.0;

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
% 3: l(x) = x'Qx

% Q = [10,0,0,0; 0,10,0,0; 0,0,0.1,0; 0,0,0,0.1]/10;
% data0 = QuadCost(g,Q);

% data0 = shapeCylinder(g, [3 4], [0; 0; 0 ; 0], 0) ;
% data_filename = 'V_g=0_5s_u_[1_0.1=d]_grid_-3_pos.mat';
% data_filename ='V_g=0_5s_u_[pi2_0.3=d]_grid_-3_pos.mat';
data_filename ='V_4D_g=0_5s_u_[pi_0.3=d]_grid_-3_Q_pos.mat';
data0 = load(data_filename, 'dataX').dataX - load(data_filename, 'TEB').TEB - 0.1;
min(data0, [], 'all')

% data_filename = 'V_g=0_5s_u_[1_0.3=d]_grid_-3_pos.mat';
% data2_filename = 'V_g0.0_5s_u_[pi2_0.3=d]_grid_-3_pos.mat';
% data0 = load(data_filename, 'dataX').dataX;
% min(data0, [], 'all')
% data02 = load(data2_filename, 'dataX').dataX;
% min(data02, [], 'all')
% 
% [g2D,data02D]=proj(g,data0,[0 0 1 1], 'min');
% [~,data022D]=proj(g,data02,[0 0 1 1], 'min');
% visFuncIm(g2D, data02D,'r',0.3);
% hold on
% visFuncIm(g2D, data022D,'c',0.3);
% visSetIm(g2D, data02D,'b', 0.1);
% figure
% level= 0:0.1:1;
% for i = 1:length(level)
%     extra.LineWidth = 1;
%     extra.LineStyle = '--';
%     visSetIm(g2D, data02D,'b', level(i), extra);
%     hold on
% end
cost_type = 'Q_pos';
cost = data0;

[dataX,tau1] = ComputeHJ(data0,cost,tau,schemeData);
TEB = min(dataX,[],'all');

% save('V_g=0.1_grid_-3_abs.mat')
% save('g_fine.mat','g')
% filename = sprintf('V_g%.1f_5s_u_[1_0.1=d]_grid_-3_%s.mat', gamma, cost_type);
filename = sprintf('V_4D_g=%.1f_10s_u_[pi_0.3=d]_grid_-3_%s.mat', gamma, cost_type);
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
HJIextraArgs.convergeThreshold = 0.02;
HJIextraArgs.stopConverge = 1;
HJIextraArgs.divergeThreshold = 8;
HJIextraArgs.ignoreBoundary = 1;
HJIextraArgs.keepLast = 1;
HJIextraArgs.makeVideo = 0;
HJIextraArgs.targetFunction = cost;
HJIextraArgs.visualize.plotData.plotDims = [1 1 0 0];
HJIextraArgs.visualize.plotData.projpt = {'min','min'};
HJIextraArgs.visualize.viewAngle = [30,45,30];

[data, tau, ~] = ...
    HJIPDE_ZGsolve(data0, tau0, schemeData, 'minCLF', HJIextraArgs);

end