function path = astar(grid, startPos, goalPos)
    % A* Pathfinding Algorithm
    
    [rows, cols] = size(grid);

    % Validate grid
if isempty(grid) || ~ismatrix(grid) || ...
        any(~isfinite(grid(:))) || ...
        any(grid(:) ~= 0 & grid(:) ~= 1)
    error('Grid must contain only 0 and 1.');
end

% Validate coordinates
if numel(startPos) ~= 2 || numel(goalPos) ~= 2
    error('Start and goal must be [row, col].');
end

if any(~isfinite(startPos)) || any(~isfinite(goalPos)) || ...
        any(startPos ~= fix(startPos)) || ...
        any(goalPos ~= fix(goalPos))
    error('Coordinates must be finite integers.');
end

if startPos(1) < 1 || startPos(1) > rows || ...
        startPos(2) < 1 || startPos(2) > cols || ...
        goalPos(1) < 1 || goalPos(1) > rows || ...
        goalPos(2) < 1 || goalPos(2) > cols
    error('Start or goal is outside the grid.');
end

if grid(startPos(1), startPos(2)) == 1
    error('Start position is blocked.');
end

if grid(goalPos(1), goalPos(2)) == 1
    error('Goal position is blocked.');
end
    
    % Matrices to track visited and pending cells (Closed & Open lists)
    closedSet = false(rows, cols); 
    
    % Matrices to store costs
    gScore = inf(rows, cols);
    gScore(startPos(1), startPos(2)) = 0; % Start cost is 0
    
    fScore = inf(rows, cols);
    fScore(startPos(1), startPos(2)) = heuristic(startPos, goalPos);
    
    % Cell array to store "where did I come from?" to reconstruct the path later
    cameFrom = cell(rows, cols);
    
    % Open list of cells to be evaluated
    openList = [startPos(1), startPos(2), fScore(startPos(1), startPos(2))];
    
    % Allowed drone movements (right, left, down, up)
    moves = [0 1; 0 -1; 1 0; -1 0]; 
    
    while ~isempty(openList)
        % 1. Choose the cell with the lowest fScore from the open list
        [~, minIdx] = min(openList(:, 3));
        current = openList(minIdx, 1:2);
        
        % 2. Did we reach the goal?
        if current(1) == goalPos(1) && current(2) == goalPos(2)
            path = reconstructPath(cameFrom, current);
            return;
        end
        
        % 3. Remove current cell from open list and add it to closed list
        % (This line is crucial to prevent infinite loops!)
        openList(minIdx, :) = [];
        closedSet(current(1), current(2)) = true;
        
        % 4. Check neighbors
        for i = 1:4
            neighbor = current + moves(i, :);
            
            if neighbor(1) > 0 && neighbor(1) <= rows && ...
               neighbor(2) > 0 && neighbor(2) <= cols && ...
               grid(neighbor(1), neighbor(2)) == 0 && ...
               ~closedSet(neighbor(1), neighbor(2))
           
                tentative_gScore = gScore(current(1), current(2)) + 1;
                
                if tentative_gScore < gScore(neighbor(1), neighbor(2))
                    cameFrom{neighbor(1), neighbor(2)} = current;
                    gScore(neighbor(1), neighbor(2)) = tentative_gScore;
                    fScore(neighbor(1), neighbor(2)) = tentative_gScore + heuristic(neighbor, goalPos);
                    
                    inOpen = ismember(openList(:,1:2), neighbor, 'rows');
                    if ~any(inOpen)
                        openList = [openList; neighbor(1), neighbor(2), fScore(neighbor(1), neighbor(2))];
                    else
                        idx = find(inOpen);
                        openList(idx, 3) = fScore(neighbor(1), neighbor(2));
                    end
                end
            end
        end
    end
    
    path = []; 
    disp('No path found!');
end
function h = heuristic(pos1, pos2)
    h = abs(pos1(1) - pos2(1)) + abs(pos1(2) - pos2(2));
end
function path = reconstructPath(cameFrom, current)
    path = current;
    while ~isempty(cameFrom{current(1), current(2)})
        current = cameFrom{current(1), current(2)};
        path = [current; path]; 
    end
end