clc;
clear;

% Define graph
graph = {
    'S', {'A', 5; 'B', 9; 'D', 6}, 5;
    'A', {'B', 3; 'G1', 9}, 7;
    'B', {'C', 1; 'A', 2}, 3;
    'C', {'S', 6; 'F', 7; 'G2', 5}, 4;
    'D', {'E', 2; 'C', 2}, 6;
    'E', {'G3', 7}, 5;
    'F', {'G3', 8}, 6;
    'G1', {}, 0; 
    'G2', {}, 0; 
    'G3', {}, 0;
    };

% Define Start and Goal nodes
startNode = 'S';
goalNodes = {'G1', 'G2', 'G3'}; % Cell array of one or more goal(s)

% Call the function
[path, cost, stepTable] = search_with_table(graph, startNode, goalNodes);

% Display the result
disp('Step-by-Step Table:');
disp(stepTable);
disp('Solution path:');
disp(path);
disp('Cost:');
disp(cost);

% ========================================

function [path, cost, stepTable] = search_with_table(graph, startNode, goalNodes)

% Get the nodes
nodes = graph(:,1);

hcosts = containers.Map(nodes, cell2mat(graph(:, 3)));

% Initialize priority queue (node, g-cost, f-cost), visited array, parent array, and costs
queue = {startNode, 0, 0, 'None'};  % Priority queue: node, g-cost, f-cost, parent
costs = Inf(1, length(nodes));  % Costs to reach each node (initialize to infinity)
costs(strcmp(nodes, startNode)) = 0;  % Cost to reach the start node is zero
parent = cell(1, length(nodes));  % Parent array to reconstruct path
visited = false(1, length(nodes));  % Visited nodes array

% Initialize step table to track each step
stepTable = table([], {}, {}, 'VariableNames', ...
    {'Step', 'Frontier', 'SelectedNode'});

stepCount = 0; % Step counter for the table
GoalFound=false;
foundGoals = {}; % Initialize an empty list to store found goals

% Main loop
while true
    % VISUAL - Before removing the selected node, capture the state of the frontier
    frontierStr = "";
    for k = 1:size(queue, 1)
        % Display f value and parent for each node in the frontier
        if k>1
            frontierStr = frontierStr  + ", ";
        end
        frontierStr = frontierStr + queue{k, 1} + "(" + num2str(queue{k, 2}) + "," + queue{k, 4} + ")";
    end

    % Check if the current node is one of the goals
    if GoalFound && length(foundGoals) == length(goalNodes)
        % Calculate the final cost
        current=GoalFoundNode;
        currentIndex = strcmp(nodes, current);
        cost = costs(currentIndex);
        % Reconstruct the path
        path = {};
        while ~isempty(current)
            path = [current, path];  % Add current node to the path
            for i = 1:length(nodes)
                if strcmp(nodes{i}, current)
                    current = parent{i};  % Move to the parent
                    break;
                end
            end
        end
        return;
    elseif isempty(queue)
        path={'NOT FOUND'};
        cost=0;
        return;
    else
        % Get the node with the lowest g-cost
        [~,minFCostidx]=min(cat(1,queue{:,3}));
        current = queue{minFCostidx, 1}; % Get the node with the lowest f-cost
        currentGCost = queue{minFCostidx, 2}; % Get the corresponding g-cost
        currentFCost = queue{minFCostidx, 3}; % Get the corresponding f-cost
        currentParent = queue{minFCostidx, 4}; % Get the parent of the current node
        queue(minFCostidx, :) = [];  % Dequeue the current node         
    end
    
    if ismember(current, goalNodes) && ~ismember(current, foundGoals)
        foundGoals{end+1} = current; % Add the goal to the list of found goals
    end

    % If all goals are found, stop the search
    if length(foundGoals) == length(goalNodes)
        GoalFound = true;
        GoalFoundNode = current; 
    end

    % Mark the current node as visited
    for i = 1:length(nodes)
        if strcmp(nodes{i}, current)
            visited(i) = true;
            break;
        end
    end

    % Find neighbors of the current node and their costs
    for i = 1:size(graph, 1)
        if strcmp(graph{i, 1}, current)
            neighbors = graph{i, 2};  % Get neighbors and costs
            break;
        end
    end
  
    % Explore neighbors
    for i = size(neighbors, 1):-1:1 % Reverse order to prepend neighbors
        neighbor = neighbors{i, 1};
        edgeCost = neighbors{i, 2};
        % Check if neighbor has not been visited or if a cheaper path is found
        for j = 1:length(nodes)
            if strcmp(nodes{j}, neighbor)
                newGCost = currentGCost + edgeCost;
                newFCost = 0;  % *depth-first* search
                if newGCost < costs(j)  % If the new cost is cheaper
                    parent{j} = current;  % Update parent
                    costs(j) = newGCost;   % Update g-cost
                    if ~ismember(neighbor, queue(:, 1))
                        queue = [{neighbor, newGCost, newFCost, current}; queue];  % first-neighbor preference
                    else
                        % Update the g and f cost in the queue if this path is cheaper
                        for k = 1:size(queue, 1)
                            if strcmp(queue{k, 1}, neighbor) && queue{k, 2} > newGCost
                                queue{k, 2} = newGCost;
                                queue{k, 3} = newFCost;
                                queue{k, 4} = current; % Update the parent in the queue
                                break;
                            end
                        end
                    end
                end
                break;
            end
        end
    end

    % VISUAL - Update step table with current state
    stepCount = stepCount + 1;
    stepTable = [stepTable; {stepCount, frontierStr, current}];
end

end

