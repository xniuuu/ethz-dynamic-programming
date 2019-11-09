% clear workspace and command window
clear;
clc;

%% PARAMETERS
alpha_dim = 500;
alpha_space = linspace(0.001, 0.999, alpha_dim);
P = [0.8, 0.2;
    0.5, 0.5;
    0.7, 0.3;
    0.4, 0.6];
P = reshape(P, 2, 2, 2);
P = permute(P, [3, 1, 2]);
G = [4, 6;
    -5, -3];
err = 1e-4;
%% VALUE ITERATION
J = max(G, [], 2);
all_actions = zeros(alpha_dim, 2);
for i = 1:alpha_dim
    iter = 0;
    alpha = alpha_space(i);
    while (1)
    [cost_to_go, actions]= ...
        max(G + alpha * reshape(sum(P .* J, 1), 2, 2).',...
        [], 2);
        iter = iter + 1;
        % Check if cost has converged
        if (max(abs(J-cost_to_go), [], 'all') / ...
                max(abs(cost_to_go), [], 'all') < err)
            J = cost_to_go;
            break;
        else
            J = cost_to_go;
        end
    end
    % Display
    disp(['Terminated after ',num2str(iter,'%9.0f'),' iterations:',...
        ' For alpha = ',num2str(alpha,'%9.2f'),...
        ', actions are ', sprintf('%d ', actions)]); 
    all_actions(i, :) = actions;
end
