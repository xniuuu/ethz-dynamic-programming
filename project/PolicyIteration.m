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

global K HOVER

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% Initializations


%policy, at the terminal state the optimal input is hover
policy = ones(K,1);
policy(TERMINAL_STATE_INDEX) = HOVER;

% corresponding cost
P_corr = zeros(K,K);
G_corr = zeros(K,1);
for i=1:K
    P_corr(i,:) = P(i,:,policy(i));
    G_corr(i) = G(i,policy(i));
end
J = linsolve((eye(476,476)-P_corr),G_corr);

%%iteration count
it = 0;

while 1
    
    %Iteration update
    it = it + 1;
    
    for i=1:K      
        policy(i) = min(G(i,:) + J'*squeeze(P(i,:,:)));
    end
    
    P_corr_new = zeros(K,K);
    G_corr_new = zeros(K,1);
    
    for i=1:K
        P_corr_new(i,:) = P(i,:,policy(i));
        G_corr_new(i) = G(i,policy(i));
    end
    
    J_new = linsolve((eye(476,476)-P_corr_new),G_corr_new);
    
    if J_new==J
        J_opt=J;
        u_opt_ind = policy;
        break
    else
        J=J_new;
    end
end
disp(it);
end


