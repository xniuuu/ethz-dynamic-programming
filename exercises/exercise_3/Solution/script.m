% script.m
%
% Matlab Script that solves Problem 4 of Problem Set 3 by applying the
% Label Correcting method and A* algorithm.
%
% Dynamic Programming and Optimal Control
% Fall 2019
% Problem Set 3, Problem 4
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Dario Brescianini
% bdario@ethz.ch
%
% --
% Revision history
% [08.10.2013, Dario Brescianini]         First version (based on old programming exercises)
%

%% Clear workspace and command window
clear all;
close all;
clc;

%% Initialize variables
load A.mat;     % Load matrix A that contains all the transition costs A(i,j) = a_ij to get from i to j.
N = length(A);  % Dimension of the problem: N = total number of nodes

%% Define start and terminal node
% Default values:
%   startNode = 1;
%   terminalNode = 100;
%
% Minimum path length (minimum total cost): 100
% Path: 1 -> 3 -> 41 -> 51 -> 100

startNode = 1;      
terminalNode = 100;

%% Label Correcting Algorithm
% Solve shortest path problem using the Label Correcting Algorithm
tic;
[optCost1, optPath1] = lca(A,startNode,terminalNode);
time1 = toc;

%% A* Algorithm
% Solve shortest path problem using the A* Algorithm
tic;
[optCost2, optPath2] = astar(A,startNode,terminalNode);
time2 = toc;

%% Display results
disp(' ');
disp('------------------------------------------------------------------');
disp('Results');
disp(['Problem with ',num2str(N),' nodes.']);
disp(['Optimal path from node ',num2str(startNode),' to ',num2str(terminalNode),':']);
disp('------------------------------------------------------------------');
disp(' ');
disp('Label Correcting Algorithm');
disp(['  Execution time: ',num2str(time1),'s.']);
disp(['  Minimum path length (minimum total cost): ',num2str(optCost1)]);
disp(['  Path: ',num2str(optPath1)]);
disp(' ');
disp('A* Algorithm');
disp(['  Execution time: ',num2str(time2),'s  (',num2str(time2/time1),' times the time for method 1).']);
disp(['  Minimum path length (minimum total cost): ',num2str(optCost2)]);
disp(['  Path: ',num2str(optPath2)]);
disp(' ');
disp('------------------------------------------------------------------');
