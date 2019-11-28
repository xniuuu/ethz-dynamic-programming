function [optCost, optPath] = astar(A,startNode,terminalNode)
% [optCost, optPath] = lca(A,startNode,terminalNode)
%
% Function template for the execution of the A* algorithm (Book Dynamic 
% Programming and Optimal Control, Bertsekes, page 87).
%
% Input:
%   A               [NxN] matrix, where the element A(i,j) = a_ij is the cost
%                   to move from node i to j.
%   startNode       Start node of desired shortest path, scalar from 1 to N.
%   terminalNode    Terminal node of desired shortest path, scalar from 1
%                   to N.
%
% Output:
%   optCost         Cost of the shortest path(s), scalar:
%   optPath         Row vector containing the shortest path, e.g. 
%                   optPath = [1 33 45 43 79 100].
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

optCost = inf;
optPath = [startNode terminalNode];

end

