function P = ComputeTransitionProbabilities(stateSpace, map)
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
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX
P = zeros(K, K, 5);
for u = [NORTH, SOUTH, EAST, WEST, HOVER]
    for i = 1:1:size(stateSpace,1)
        carry_state = stateSpace(i,3);
        stateSpace_red = stateSpace(stateSpace(:,3)==carry_state,:);
        %the stateSpace could be further reduced by introducing a filter
        %ofr the 13 attainable states
        %drone_range =
        if BoundariesCheck(i, u, stateSpace_red) || ~TreesCheck(i, u, map,stateSpace_red) 
            j = stateSpace(i,2);
            switch u
                case NORTH
                    %Probabilities with wind
                    P_Crash=                    
                    P(i,j,u) = P_WIND*(1/4)*(1-P_Tree)*(1-P_Bound)*(1-ComputeAngryResidentsCrashProbability(stateSpace(j,:), map))
                    P(i+2,j,u) = 
                    P(i+1,j-1,u) = 
                    p(i+1,j+1,u) = 
                    %Probabilities without wind
        end
    end
end

%% Take into account what happens if we start at the home base and keep
%% hovering.
%% Take into account what happens if we are at the terminal state.
end


function trees = TreesCheck(i, u, map,stateSpace)
%Function TreesCheck, check if there is a tree where we want to move.
%Inputs : 
%i :current state
%u : input (N,E,S,W or H)
%map : A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%stateSpace :A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%
%Output : true or false
global TREE
global NORTH SOUTH EAST WEST 
pos =[];
cdt = [];
pos = stateSpace(i,1:2); %actual position of the drone
[treeM, treeN] = find(map == TREE);
cdt = [isequal(ismember([treeM treeN],pos + [0 1],'rows'), zeros(length([treeM treeN]),1));
    isequal(ismember([treeM treeN],pos - [0 1],'rows'), zeros(length([treeM treeN]),1));
    isequal(ismember([treeM treeN],pos + [1 0],'rows'), zeros(length([treeM treeN]),1));
    isequal(ismember([treeM treeN],pos - [1 0],'rows'), zeros(length([treeM treeN]),1))];
if cdt(1) == 0 && u == NORTH %check if there is a tree above
    fprintf('Drone crashes into a tree');
    trees = true;
    return 
elseif cdt(2) == 0 && u == SOUTH %check if there is a tree below
    fprintf('Drone crashes into a tree');
    trees = true;
    return
elseif cdt(3) == 0 && u == EAST %check if there is a tree on the right
    fprintf('Drone crashes into a tree');
    trees = true;
    return
elseif cdt(4) == 0 && u == WEST %check if there is a tree on the left
    fprintf('Drone crashes into a tree');
    trees = true;
    return
else 
    fprintf('no tree');
    trees = false;
end

end


function boundaries = BoundariesCheck(i, u, stateSpace)
%note : does not check if the wind pushes the drone outside the boundaries
%but not sure if we need that case. Doesn't see if there are trees at the 
%boundary either


%Function boundaries, check if the drone stays inside the map according to the input.
%Inputs : 
%i :current state
%u : input (N,E,S,W or H)
%stateSpace :A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%
%Output : true or false
global NORTH SOUTH EAST WEST 
pos =[];
cdt = [];
%looking for the position of the drone
pos = stateSpace(i,1:2);%gives the position of the drone
%defining the boundaries
boundMNorth = [2:14 ; 20*ones(1,13)]';
boundMSouth = [2:14; ones(1,13)]';
boundNEast = [15*ones(1,18);2:19]';%not to have twice the corners
boundNWest = [ones(1,18); 2:19]';
boundCorners = [1 1;1 20;15 1;15 20];

%looking where is the drone
cdt = [isequal(ismember(boundMNorth,pos,'rows'), zeros(length(boundMNorth),1));
    isequal(ismember(boundMSouth,pos,'rows'), zeros(length(boundMSouth),1));
    isequal(ismember(boundNEast,pos,'rows'), zeros(length(boundNEast),1));
    isequal(ismember(boundNWest,pos,'rows'), zeros(length(boundNWest),1));
    isequal(ismember(boundCorners,pos,'rows'), zeros(length(boundCorners),1))];
disp(cdt)
%still looking where the drone is and if the input is possible
boundaries = 0;
if cdt(1) == 0 
    bound = boundMNorth;
    fprintf('Drone in north bound');
    if u == NORTH
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return 
    end   
elseif cdt(2) == 0
        bound = boundMSouth;
        fprintf('Drone in south bound');
        if u == SOUTH
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return
        end
    
elseif cdt(3) == 0
        bound = boundNEast;
        fprintf('Drone in east bound');
        if u == EAST
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return
        end
        
elseif cdt(4) == 0
        bound = boundNWest;
        fprintf('Drone in west bound');
        if u == WEST
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return
        end
        
elseif cdt(5) == 0
        bound = boundCorners;
        fprintf('Drone in a corner');
        corner = ismember(boundCorners,pos,'rows');
        if corner(1) == 1 && (u == WEST || u == SOUTH)
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return 
        elseif corner(2) == 1 && (u == NORTH || u == WEST)
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return
        elseif corner(3) == 1 && (u == SOUTH || u == EAST)
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return
        elseif corner(4) == 1 && (u == NORTH || u == EAST)
            fprintf('Drone outside boundaries, crashes');
            boundaries = true;
            return
        end
end
if boundaries ==0
    fprintf('Drone OK');
    boundaries = false;
    return
end


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
Pr =  (d <= R).' * (GAMMA ./ (d + 1));
end

function P_Tree = ComputeCrashProbabilityWind(currentState,map)
%Function that computes the probability of crashing due to a tree or the boundary in one of
%the states attainable by wind
end

