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
global P_WIND
global DROP_OFF 
global NORTH SOUTH EAST WEST HOVER
global K Nc
G = Inf(K, 5);
delta = [NORTH, [0, 1];
    SOUTH, [0, -1];
    EAST, [1, 0];
    WEST, [-1, 0];
    HOVER, [0, 0]];
[tX, tY] = find(map == DROP_OFF);
T = state2ind([tX, tY, 1], stateSpace);
G(T, :) = 0;
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
                    pNextState = 0;
                    pNeighbour = ComputeAngryResidentsCrashProbability(...
                        nextStates(k, :), map);
                    if k == HOVER
                        pNextState =  (1 - pNeighbour) * (1 - P_WIND);
                    else
                        pNextState =  (1 - pNeighbour) * P_WIND * 0.25;
                    end
                    survivalProbability = survivalProbability + pNextState;
                    assert(survivalProbability <= 1,...
                        'Probabilities are <= 1');
                    if AboveHomeBaseWithoutPacakge(nextStates(k, :), map)
                        driftedToHomeBaseProbability = ...
                            driftedToHomeBaseProbability + pNextState;
                    end
                end
            end
            G(i,u) = survivalProbability * 1 + ...
                (1 - survivalProbability) * Nc;
        end
    end
end
end

function above = AboveHomeBaseWithoutPacakge(state, map)
global BASE
above = (map(state(1), state(2)) == BASE) & (state(3) == 0);
end

function trees = TreesCollision(state, map)
%Function TreesCheck, check if there is a tree where we want to move.
%Inputs :
%state : current state
%map : A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%Output : true or false
global TREE
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
global SHOOTER GAMMA R
[shootersX, shootersY] = find(map == SHOOTER);
shootersCoordinates = [shootersX, shootersY];
d = vecnorm((repmat(currentState(1:2), size(shootersCoordinates, 1), 1) -...
    shootersCoordinates).', 1).';
Pr = (d <= R) .* (GAMMA ./ (d + 1));
Pr = 1 - prod(1 - Pr);
assert(Pr <= 1, 'Neighbour''s probability is <= 1');
end

function ind = state2ind(state, stateSpace)
ind = find(sum(stateSpace == state, 2) == 3);
end

