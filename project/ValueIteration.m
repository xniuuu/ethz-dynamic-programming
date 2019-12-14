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
    global K 
    global NORTH SOUTH EAST WEST HOVER
    global TERMINAL_STATE_INDEX
    u_opt_ind = HOVER * ones(K, 1);
    u_opt_ind(TERMINAL_STATE_INDEX) = HOVER;
    % Initializing greedily.
    J_opt = min(G, [], 2);
    costToGo = ones(K, 1);
    err = 1e-5;
    while 1
        values = Inf(K, 5);
        for u = [NORTH SOUTH EAST WEST HOVER]
            values(:, u) = G(:, u) + P(:, :, u) * J_opt;
        end
        [costToGo, u_opt_ind] = min(values, [], 2);
        if norm(costToGo - J_opt, inf) < err
            J_opt = costToGo;
            break;
        else
            J_opt = costToGo;
        end
    end

end