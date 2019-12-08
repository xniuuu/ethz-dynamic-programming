function G = ComputeStageCosts(stateSpace, map)
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [P,PSurvive,PCrash]= ComputeTransitionProbabilitiesAugmented(stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map)
%   computes the transition probabilities between all states in the state
%   space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
% global P_WIND
% global DROP_OFF BASE 
% global NORTH SOUTH EAST WEST HOVER
% global K
P = zeros(K, K, 5);
PSurvive = zeros(K,1,5);
PCrash = zeros(K,1,5);
delta = [NORTH, [0, 1];
    SOUTH, [0, -1];
    EAST, [1, 0];
    WEST, [-1, 0];
    HOVER, [0, 0]];
[baseX, baseY] = find(map == BASE);
baseInd = state2ind([baseX, baseY, 0], stateSpace);
[tX, tY] = find(map == DROP_OFF);
T = state2ind([tX, tY, 1], stateSpace);
P(T, T, :) = 1;
for i = 1:size(stateSpace,1)
    if (i == T)
        continue;
    end
    for u = [NORTH, SOUTH, EAST, WEST, HOVER]
        % Augmenting 0 to delta so we don't change the 'package carrying'
        % state.
        stateAfterAction = stateSpace(i, :) + [delta(u, 2:end), 0];
        if ~OutOfBoundaries(stateAfterAction, size(map)) &...
                ~TreesCollision(stateAfterAction, map)
            nextStates = repmat(stateAfterAction, size(delta, 1), 1) +...
                [delta(:, 2:end), zeros(size(delta, 1), 1)];
            % The probability of surviving the current transition.
            survivalProbability = 0;
            % The probabaility of arriving above the home-base without 
            % actually dying along the way.
            driftedToHomeBaseProbability = 0;
            for k = [NORTH, SOUTH, EAST, WEST, HOVER]
                if  ~OutOfBoundaries(nextStates(k, :), size(map)) &...
                        ~TreesCollision(nextStates(k, :), map)
                    if Pickup(nextStates(k, :), map)
                        j = state2ind([nextStates(k, 1:2), 1], stateSpace);
                    else 
                        j = state2ind(nextStates(k, :), stateSpace);
                    end
                    pNeighbour = ComputeAngryResidentsCrashProbability(...
                        nextStates(k, :), map);
                    if k == HOVER
                        P(i, j, u) =  (1 - pNeighbour) * (1 - P_WIND);
                    else
                        P(i, j, u) =  (1 - pNeighbour) * P_WIND * 0.25;
                    end
                    survivalProbability = survivalProbability + P(i, j, u);
                    assert(survivalProbability <= 1,...
                        'Probabilities are <= 1');
                    if AboveHomeBaseWithoutPacakge(nextStates(k, :), map)
                        driftedToHomeBaseProbability = ...
                            driftedToHomeBaseProbability + P(i, j, u);
                    end
                end
            end
            % Either there was no wind, but the neighbours killed you where
            % you planned to go (only via the action), or you arrived to
            % some other position through the wind and survived.
            P(i, baseInd, u) = 1 - survivalProbability +...
                driftedToHomeBaseProbability;
            PSurvive(i, baseInd, u) = driftedToHomeBaseProbability;
            PCrash(i, baseInd, u) = P(i, baseInd, u)-PSurvive(i, baseInd, u);
        end
    end
end
end

function pickup = Pickup(state, map)
% global PICK_UP
pickup = map(state(1), state(2)) == PICK_UP &...
    state(3) == 0;
end

function above = AboveHomeBaseWithoutPacakge(state, map)
% global BASE
above = (map(state(1), state(2)) == BASE) & (state(3) == 0);
end

function trees = TreesCollision(state, map)
%Function TreesCheck, check if there is a tree where we want to move.
%Inputs :
%state : current state
%map : A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%Output : true or false
% global TREE
trees = (map(state(1), state(2)) == TREE);
end


function boundaries = OutOfBoundaries(state, mapSize)
%Function boundaries, check if the drone stays inside the map according to the input.
%Inputs :
%state : current state
%mapSize: the map dimensions.
%
%Output : true or false
coordinates = state(1:2);
boundaries = any(coordinates > mapSize) || any(coordinates <= 0);
end

function Pr = ComputeAngryResidentsCrashProbability(currentState, map)
%Function ComputeAngryResidentsCrashProbability, commputes the probability
%of crashing due to angry residents.
%Inputs :
%currentState :current state (x,y,psi) of the drone.
%map : the drones world (including angry residents and )
%
%
%Output : The probabaility of crashing due to angry residents.
% global SHOOTER GAMMA R
[shootersX, shootersY] = find(map == SHOOTER);
shootersCoordinates = [shootersX, shootersY];
d = vecnorm((repmat(currentState(1:2), size(shootersCoordinates, 1), 1) -...
    shootersCoordinates).', 1).';
Pr = (d <= R) .* (GAMMA ./ (d + 1));
Pr = 1 - prod(1 - Pr);
assert(Pr <= 1, 'Neighbour''s probability is <= 1');
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


[P,PSurvive,PCrash]= ComputeTransitionProbabilitiesAugmented(stateSpace, map);
    G = inf(K,5);
%Setting the cost for terminal state
    G(TERMINAL_STATE_INDEX,:)=0;
    N = 1; %cost when no crash
%looking for base index
    [baseX, baseY] = find(map == BASE);
    baseInd = state2ind([baseX, baseY, 0], stateSpace);
    
    for i = 1:size(stateSpace,1)
        if (i==TERMINAL_STATE_INDEX)
            continue;
        end
        for u = [NORTH SOUTH EAST WEST HOVER]
            G(i,u) = sum(P(i,1:end ~= baseInd,u))+ sum(PSurvive(i,baseInd,u))+PCrash(i,baseInd,u)*Nc;
        if G(i,u)==0
            G(i,u)=inf;%impossible states
        end
        end
    end
    
    
    function ind = state2ind(state, stateSpace)
        ind = find(sum(stateSpace == state, 2) == 3);
    end

end

