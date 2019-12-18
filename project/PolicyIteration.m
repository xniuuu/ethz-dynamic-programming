function [J_opt, u_opt_ind] = PolicyIteration(P, G)
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
    global NORTH SOUTH EAST WEST HOVER
    global K 
    global TERMINAL_STATE_INDEX
    % Hopefully get a proper initial policy.
    G(TERMINAL_STATE_INDEX, :) = [];
    P(TERMINAL_STATE_INDEX, :, :) = [];
    P(:, TERMINAL_STATE_INDEX, :) = [];
    [J_opt, u_opt_ind] = min(G, [], 2);
    u_opt_ind = HOVER * ones(K - 1, 1);
    Jh = ones(K - 1, 1);
    while 1
        [A, b] = ResolveLinearSystem(u_opt_ind, P, G);
        Jh = A \ b;
%         Jh = ApproximateNextCostToGo(P, G, u_opt_ind, J_opt, 100);
        values = Inf(K - 1, 5);
        for u = [NORTH SOUTH EAST WEST HOVER]
            values(:, u) = G(:, u) + P(:, :, u) * Jh;
        end
        [~, u_opt_ind] = min(values, [], 2);
        if norm(Jh - J_opt, inf) <= 1e-5
            J_opt = Jh;
            break;
        end
        J_opt = Jh;
    end
    J_opt = [J_opt(1:(TERMINAL_STATE_INDEX - 1));
    0;
    J_opt(TERMINAL_STATE_INDEX:end)];
    u_opt_ind = [u_opt_ind(1:(TERMINAL_STATE_INDEX - 1));
        HOVER;
        u_opt_ind(TERMINAL_STATE_INDEX:end)];
end

function [A, b] = ResolveLinearSystem(...
    policy,...
    transitionProbabilitiesTable,...
    stageCosts)
    global K
    P = zeros(K - 1, K - 1);
    b = zeros(K - 1, 1);
    for i = 1:(K - 1)
        P(i, :) = transitionProbabilitiesTable(i, :, policy(i));
        b(i) = stageCosts(i, policy(i));
    end
    A = eye(K - 1, K - 1) - P;
end

function costToGo = ApproximateNextCostToGo(P, G, policy, J, n)
    global NORTH SOUTH EAST WEST HOVER
    costToGo = Inf(size(J));
    for iter = 1:n
        for u = [NORTH SOUTH EAST WEST HOVER]
            costToGo(policy == u) =...
                G(policy == u, u) + P(policy == u, :, u) * J;
        end
        if norm(J - costToGo, inf) / norm(J, inf) <= 1e-3
            break;
        end
        J = costToGo;
    end
end
