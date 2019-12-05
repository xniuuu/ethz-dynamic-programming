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
