close all; clear all; clc

Map = importdata('Map.mat');

x = 1;
y = 3; 
senseRange = 3;
TEB = [2,1];
dim = 2;

[SenseMap,old_map] = SenseEnv(x,y,senseRange,Map,TEB,dim);