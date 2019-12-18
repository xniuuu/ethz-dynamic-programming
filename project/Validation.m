%% Note: this code only works after running main.mat at least once.
%% Validate P matrix
clc;
clear P;
load('exampleP.mat');
PGroundTruth = P;
P = ComputeTransitionProbabilities(stateSpace, map);
for u = [NORTH, SOUTH, EAST, WEST, HOVER]
    assert(norm(PGroundTruth(:, :, u) - P(:, :, u), inf) <= 1e-4);
end
disp('Passed successfully the TransitionProbabilities matrix computation test');
%% Validate G matrix
clear G;
load('exampleG.mat');
GGroundTruth = G;
G = ComputeStageCosts(stateSpace, map);
diff = GGroundTruth - G;
assert(all(isfinite(diff) == isfinite(GGroundTruth), 'all'),...
    'NaN Values are not the same');
diff(~isfinite(diff)) = 0;
assert(norm(diff, inf) <= 1e-4);
disp('Passed successfully the StageCosts matrix computation test');
%% Validate The Algorithms Implementation (V* is unique) 
[JLp, ~] = LinearProgramming(P, G);
[JVi, ~] = ValueIteration(P, G);
[JPi, ~] = PolicyIteration(P, G);
assert(norm(range([JLp, JVi, JPi], 2), inf) < 1e-3,...
    'Discrepency between algorithms.');
disp('Passed successfully the cost uniqueness test');