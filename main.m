% source folder
addpath(genpath(fileparts(which('main'))));

% init
clear all;
close all;
clc;

% run naive RRT
rrt = RRTPlanner();
%rrt.setIncrement(1);
rrt.solve;