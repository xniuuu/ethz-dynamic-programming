function [J_opt, u_opt_ind] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

    global K
    global NORTH SOUTH EAST WEST HOVER
    global TERMINAL_STATE_INDEX
    A = ConstructConstraintsMatrix(P);
    b = ConstructConstraintsVector(G);
    f = ones(K - 1, 1);
    % Minus A because we're maximizing.
    % Filter out inf costs and their corresponding transitions.
    A = A(b ~= inf, :);
    b = b(b ~= inf);
    J_opt = linprog(-f, sparse(A), b, [], [],...
        zeros(size(f)), Inf(size(f)));
    J_opt = [J_opt(1:(TERMINAL_STATE_INDEX - 1));
        0;
        J_opt((TERMINAL_STATE_INDEX):end)];
    V = zeros(K, 5);
    for u = [NORTH, SOUTH, EAST, WEST, HOVER]
        V(:, u) = G(:, u) + P(:, :, u) * J_opt;
    end
    [~, u_opt_ind] = min(V, [], 2);
end

function b = ConstructConstraintsVector(G)
    global K TERMINAL_STATE_INDEX
    b = reshape(G(1:end ~= TERMINAL_STATE_INDEX, :).',...
        (K - 1) * 5, 1);
end

function A = ConstructConstraintsMatrix(P)
    global K TERMINAL_STATE_INDEX
    % Filter out the terminal state.
    PNew = P(1:end ~= TERMINAL_STATE_INDEX,...
        1:end ~= TERMINAL_STATE_INDEX, :);
    lhs = eye(K - 1, K - 1);
    lhs = repmat(lhs, 1, 1, 5);
    lhs = lhs - PNew;
    A = permute(lhs, [3, 1, 2]);
    A = reshape(A, (K - 1) * 5, K - 1);
end

