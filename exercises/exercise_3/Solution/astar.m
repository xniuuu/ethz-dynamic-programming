function [optCost, optPath] = astar(A,startNode,terminalNode)
% [optCost, optPath] = lca(A,startNode,terminalNode)
%
% Executes A* algorithm (Book Dynamic Programming and Optimal
% Control, Bertsekes, page 87) using the depth-first method.
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

%% Initialization
N = length(A);      % Dimension of the problem: N = total number of nodes

d = ones(1,N)*inf;  % Vector holding label d for each node. d(i) represents
                    % the shortest path found so far from start node to i.
d(startNode) = 0;

parent = ones(1,N)*inf; % Vector containing the parent of the shortest path
                        % found so far for each node.
parent(startNode) = 0;

OPEN = zeros(1,N);  % List cotaining all the nodes that are currently 
                    % active in the sense that they are candidates for 
                    % further examination (candidates list).
pointerOPEN = 1;    % Pointer which always points to the last element in OPEN.
OPEN(pointerOPEN) = startNode;

UPPER = inf;    % Label dt, representing the shortest path to the end found so far.


%% Check start and terminal node
% Make sure that the start and terminal node are valid.
if startNode == terminalNode
    optCost = 0;
    optPath = [startNode terminalNode];
    return;         % Done.
end

if (startNode > N || terminalNode > N)
    optCost = inf;
    optPath = [startNode terminalNode];
    return;         % Done.
end


%% Execute algorithm
while 1
    % STEP 1: Remove a node i from OPEN and for each child j of i, execute STEP 2.
    i = OPEN(pointerOPEN);
    OPEN(pointerOPEN) = 0;
    pointerOPEN = pointerOPEN - 1;
    
    children = find(~isinf(A(i,:)) == 1);
    children(children == i) = [];
    
    for j = children
        % STEP 2: If d_i + a_ij < and  d_i + a_ij + h_j < UPPER, 
        % set d_j = d_i + a_ij and set i to be the parent of j.
        if (d(i) + A(i,j) < d(j) && d(i) + A(i,j) + abs(j-terminalNode) < UPPER)
            d(j) = d(i) + A(i,j);

            parent(j) = i;
            
            % In addition, if j ~= t, place j in OPEN if it is not already
            % in OPEN, while if j == t, set UPPER to the new value d_i +
            % a_it of d_t
            if (j ~= terminalNode)
                if ~(OPEN == j)
                    pointerOPEN = pointerOPEN + 1;
                    OPEN(pointerOPEN) = j;
                end
            else
                UPPER = d(j);
            end
        end
    end
    
    % STEP 3: If OPEN is empty, terminate; else go to STEP 1.
    if (~pointerOPEN)
        break;
    end
end

%% DONE.
% UPPER is equal to the cost of the shortest path.
optCost = UPPER;

%% Construct shortest path
% Start at terminal node and, for each node, take its parent node until we
% find ourselves at the start node.
optPath = terminalNode;
while optPath(end) ~= startNode
    optPath(end + 1) = parent(optPath(end));
end
optPath = fliplr(optPath);  % Reverse path: startNode -> terminalNode

end