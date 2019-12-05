%% Note: this code only works after running main.mat at least once.
%% Validate P matrix
clc;
clear P;
load('exampleP.mat');
PGroundTruth = P;
P = ComputeTransitionProbabilities(stateSpace, map);
for u = [NORTH, SOUTH, EAST, WEST, HOVER]
    [row, col] = find(abs(PGroundTruth(:, :, u) - P(:, :, u)) > 1e-4);
    disp(u);
    disp([row, col]);
end

%% Validate G matrix
clc;
clear G;
load('exampleG.mat');
GGroundTruth = G;
G = ComputeStageCosts(stateSpace, map);
assert(abs(GGroundTruth - G) > 1e-4,...
    'G matrix is wrong.');

%% Validate The Algorithms Implementation (V* is unique) 
[JLp, ~] = LinearProgramming(P, G);
[JVi, ~] = ValueIteration(P, G);
[JPi, ~] = PolicyIteration(P, G);
assert(range([JLp, JVi, JPi]) < 1e-3,...
    'Discrepency between algorithms.');