clc;
clear;

% Define graph
graph = {
     'Arad', {'Sibiu', 140; 'Zerind', 75; 'Timisoara', 118}, 366;
     'Zerind', {'Arad', 75; 'Oradea', 71}, 374;
     'Oradea', {'Zerind', 71; 'Sibiu', 151}, 380;
     'Timisoara', {'Arad', 118; 'Lugoj', 111}, 329;
     'Lugoj', {'Timisoara', 111; 'Mehadia', 70}, 244;
     'Mehadia', {'Lugoj', 70; 'Drobeta', 75}, 241;
     'Drobeta', {'Mehadia', 75; 'Craiova', 120}, 242;
     'Craiova', {'Drobeta', 120; 'R.Vilcea', 146; 'Pitesti', 138}, 160;
     'Sibiu', {'Arad', 140; 'Oradea', 151; 'R.Vilcea', 80; 'Fagaras', 99}, 253;
     'R.Vilcea', {'Sibiu', 80; 'Craiova', 146; 'Pitesti', 97}, 193;
     'Fagaras', {'Sibiu', 99; 'Bucharest', 211}, 176;
     'Pitesti', {'R.Vilcea', 97; 'Craiova', 138; 'Bucharest', 101}, 100;
     'Bucharest', {'Fagaras', 211; 'Pitesti', 101; 'Urziceni', 85; 'Giurgiu', 90}, 0;
     'Giurgiu', {'Bucharest', 90}, 77;
     'Urziceni', {'Bucharest',85; 'Hirsova', 98;'Vaslui', 142}, 80;
     'Hirsova', {'Urziceni', 98; 'Eforie', 86}, 151;
     'Eforie', {'Hirsova', 86}, 161;
     'Vaslui', {'Urziceni', 142; 'Iasi', 92}, 199;
     'Iasi', {'Vaslui', 92; 'Neamt', 87}, 226;
     'Neamt', {'Iasi', 87}, 234
    };

% Define Start and Goal nodes
startNode = 'Arad';
goalNodes = {'Bucharest'}; % Cell array of one or more goal(s)
% goalNodes = {'Eforie','Vaslui'};

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

hcosts = graph(:, 3);
hcosts_table = cell2table(hcosts, 'VariableNames', {'Values'});
hcosts_table.Properties.RowNames = nodes;

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
    if GoalFound
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
        % Get the node with the lowest f-cost
        [~,minFCostidx]=min(cat(1,queue{:,3}));
        current = queue{minFCostidx, 1}; % Get the node with the lowest f-cost
        currentGCost = queue{minFCostidx, 2}; % Get the corresponding g-cost
        currentFCost = queue{minFCostidx, 3}; % Get the corresponding f-cost
        currentParent = queue{minFCostidx, 4}; % Get the parent of the current node
        queue(minFCostidx, :) = [];  % Dequeue the current node        
    end
    if ismember(current, goalNodes)
        GoalFound=true;
        GoalFoundNode=current;
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
    for i = 1:size(neighbors, 1)
        neighbor = neighbors{i, 1};
        edgeCost = neighbors{i, 2};
        % Check if neighbor has not been visited or if a cheaper path is found
        for j = 1:length(nodes)
            if strcmp(nodes{j}, neighbor)
                newGCost = currentGCost + edgeCost;
                newHCost = hcosts_table{neighbor, 1};
                newFCost = newGCost + newHCost;  % f = g + h "A*" search
                if newGCost < costs(j)  % If the new cost is cheaper
                    parent{j} = current;  % Update parent
                    costs(j) = newGCost;   % Update g-cost
                    if ~ismember(neighbor, queue(:, 1))
                        queue(end + 1, :) = {neighbor, newGCost, newFCost, current};  % Enqueue the neighbor with the new g and f costs and parent
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

