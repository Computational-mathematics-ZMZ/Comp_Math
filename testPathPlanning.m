clc;
clear;
close all;
fprintf('Starting A* path-planning tests with visualizations...\n\n');

%% Test 1: Open grid
grid = zeros(20,20);
startPos = [2,2];
goalPos = [19,19];
path = astar(grid,startPos,goalPos);
distance = calculatePathLength(path);
assert(~isempty(path), 'Open-grid test failed: no path found.');
assert(isequal(path(1,:),startPos), 'Path does not start correctly.');
assert(isequal(path(end,:),goalPos), 'Path does not end correctly.');
assert(distance == 34, 'Expected open-grid distance is 34 steps.');
fprintf('PASS: Open grid, distance = %d steps.\n',distance);
plotTestResult(grid, startPos, goalPos, path, 'Test 1: Open Grid');

%% Test 2: Grid with cross-shaped obstacles
grid = zeros(20,20);
grid(5:15,10) = 1;
grid(10,5:15) = 1;
startPos = [2,2];
goalPos = [19,19];
path = astar(grid,startPos,goalPos);
distance = calculatePathLength(path);
assert(~isempty(path), 'Obstacle test failed: no path found.');
pathIndices = sub2ind(size(grid),path(:,1),path(:,2));
assert(all(grid(pathIndices) == 0), ...
    'Obstacle test failed: path crosses an obstacle.');
assert(all(path(:,1) >= 1 & path(:,1) <= 20), ...
    'Path contains an invalid row.');
assert(all(path(:,2) >= 1 & path(:,2) <= 20), ...
    'Path contains an invalid column.');
steps = abs(diff(path,1,1));
assert(all(sum(steps,2) == 1), ...
    'Path contains an invalid movement.');
fprintf('PASS: Obstacle avoidance, distance = %d steps.\n',distance);
plotTestResult(grid, startPos, goalPos, path, 'Test 2: Obstacle Avoidance');

%% Test 3: Goal completely surrounded by obstacles
grid = zeros(20,20);
startPos = [2,2];
goalPos = [19,19];
grid(18,19) = 1;
grid(20,19) = 1;
grid(19,18) = 1;
grid(19,20) = 1;
path = astar(grid,startPos,goalPos);
distance = calculatePathLength(path);
assert(isempty(path), ...
    'No-path test failed: a path was returned.');
assert(distance == 0, ...
    'No-path test failed: distance should be zero.');
fprintf('PASS: No-path test.\n');
plotTestResult(grid, startPos, goalPos, path, 'Test 3: Goal Surrounded by Obstacles');

%% Test 4: Start equals goal
grid = zeros(20,20);
startPos = [2,2];
goalPos = [2,2];
path = astar(grid,startPos,goalPos);
distance = calculatePathLength(path);
assert(~isempty(path), 'Same-start-goal test failed.');
assert(size(path,1) == 1, ...
    'Path should contain exactly one cell.');
assert(distance == 0, ...
    'Path length should be zero when start equals goal.');
fprintf('PASS: Start equals goal.\n');
plotTestResult(grid, startPos, goalPos, path, 'Test 4: Start Equals Goal');

%% Test 5: Blocked start must produce an error
grid = zeros(20,20);
grid(2,2) = 1;
errorCaught = false;
try
    astar(grid,[2,2],[19,19]);
catch
    errorCaught = true;
end
assert(errorCaught, 'Blocked-start test failed: no error was raised.');
fprintf('PASS: Blocked start validation.\n');
plotTestResult(grid, [2,2], [19,19], [], 'Test 5: Blocked Start (Error Case)');

%% Test 6: Out-of-bounds coordinates must produce an error
grid = zeros(20,20);
errorCaught = false;
try
    astar(grid,[0,2],[19,19]);
catch
    errorCaught = true;
end
assert(errorCaught, ...
    'Invalid-coordinate test failed: no error was raised.');
fprintf('PASS: Invalid-coordinate validation.\n');
plotTestResult(grid, [0,2], [19,19], [], 'Test 6: Out-of-Bounds (Error Case)');

%% Final result
fprintf('\nAll A* tests passed successfully.\n');

%% Helper Function for Visualization
function plotTestResult(grid, startPos, goalPos, path, testTitle)
    figure('Name', testTitle, 'Color', 'white', 'Position', [100, 100, 600, 500]);
    
    dispGrid = grid;
    
    % Mark path if available
    if ~isempty(path) && size(path,1) > 1
        for i = 1:size(path,1)
            r = path(i,1); c = path(i,2);
            if r >= 1 && r <= size(grid,1) && c >= 1 && c <= size(grid,2)
                if dispGrid(r,c) == 0
                    dispGrid(r,c) = 4;
                end
            end
        end
    end
    
    % Mark start and goal if within bounds
    if startPos(1) >= 1 && startPos(1) <= size(grid,1) && startPos(2) >= 1 && startPos(2) <= size(grid,2)
        dispGrid(startPos(1), startPos(2)) = 2;
    end
    if goalPos(1) >= 1 && goalPos(1) <= size(grid,1) && goalPos(2) >= 1 && goalPos(2) <= size(grid,2)
        dispGrid(goalPos(1), goalPos(2)) = 3;
    end
    
    imagesc(dispGrid);
    
    % Custom Colormap (0: Free, 1: Obstacle, 2: Start, 3: Goal, 4: Path)
    customMap = [
        1.0, 1.0, 1.0;  % 0: Free (White)
        0.3, 0.3, 0.3;  % 1: Obstacle (Dark Gray)
        0.0, 0.7, 0.0;  % 2: Start (Green)
        0.8, 0.0, 0.0;  % 3: Goal (Red)
        0.0, 0.4, 0.8   % 4: Path (Blue)
    ];
    colormap(customMap);
    caxis([0 4]);
    
    set(gca, 'XTick', 0.5:1:size(grid,2)+0.5, 'YTick', 0.5:1:size(grid,1)+0.5, ...
        'GridLineStyle', '-', 'XGrid', 'on', 'YGrid', 'on', ...
        'XTickLabel', [], 'YTickLabel', [], 'LineWidth', 1);
    
    axis equal tight;
    title(testTitle, 'FontSize', 14, 'FontWeight', 'bold');
    
    annotation('textbox', [0.15, 0.01, 0.7, 0.05], 'String', ...
        'Legend: [ ] Free  [ ] Obstacle  [ ] Start  [ ] Goal  [ ] Path', ...
        'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end