function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K NORTH SOUTH EAST WEST HOVER
global TERMINAL_STATE_INDEX
policy = HOVER * ones(K, 1);
policy(TERMINAL_STATE_INDEX) = HOVER;
% Initializing greedily.
J = min(G, [], 2); 
cost_to_go = ones(K, 1);
err = 1e-5;
while 1
    values = Inf(K, 5);
    for u = [NORTH SOUTH EAST WEST HOVER]
        values(:, u) = G(:, u) + P(:, :, u) * J;
    end
    [cost_to_go, policy] = min(values, [], 2);
    if (max(abs(J - cost_to_go))) / max(abs(cost_to_go)) < err
        J = cost_to_go;
        break;
    else
        J = cost_to_go;
    end
end
J_opt = cost_to_go;
u_opt_ind = policy;
end