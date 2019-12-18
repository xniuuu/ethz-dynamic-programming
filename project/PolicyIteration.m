function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
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

global K HOVER NORTH EAST SOUTH WEST

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% Initializations

%Hover is a proper policy
%Ignoring end state cause P = 1
% corresponding cost
% P_corr = P(1:end ~=TERMINAL_STATE_INDEX,1:end ~=TERMINAL_STATE_INDEX,HOVER);
% G_corr = G(1:end ~=TERMINAL_STATE_INDEX,HOVER);
% J =(eye(size(P_corr,1),size(P_corr,1))-P_corr)\G_corr;

%%iteration count
it = 0;
policy = HOVER.*ones(K-1,1);
P = P(1:end ~=TERMINAL_STATE_INDEX,1:end ~=TERMINAL_STATE_INDEX,:);
G = G(1:end ~=TERMINAL_STATE_INDEX,:);
J = ones(K-1,1);
P_corr= zeros(K-1,K-1);
G_corr = zeros(K-1,1);
while 1
    
    %Iteration update
    it = it + 1;
    
    for i=1:K-1
        P_corr(i,:) = P(i,:,policy(i));
        G_corr(i) = G(i,policy(i));
    end
    
    J_new = (eye(size(P_corr,1),size(P_corr,1))-P_corr)\G_corr;
    
    if J_new==J
        k=TERMINAL_STATE_INDEX-1;
        J_opt=[J(1:k) ; 0;J(k+1:end)];
        u_opt_ind = [policy(1:k) ; HOVER ; policy(k+1:end)];
        break
    else
        J=J_new;
    end
    
    values = Inf(K-1, 5);
    for u = [NORTH SOUTH EAST WEST HOVER]
        values(:, u) = G(:, u) + P(:, :, u) * J;
    end
    [cost_to_go, policy] = min(values, [], 2);
end
disp(it);
end


