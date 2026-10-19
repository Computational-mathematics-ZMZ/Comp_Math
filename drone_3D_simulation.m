clc;
clear;
close all;

%% 1. ENVIRONMENT SETTINGS
N = 10;
H = 4;

startPoint = [1 1 1];
goalPoint  = [9 9 3];

% Obstacles: [xmin xmax ymin ymax zmin zmax]
obstacles = [
    3 4  3 4  1 2;
    5 6  6 7  1 3;
    7 8  3 4  1 2;
    2 2  7 8  1 3;
    8 8  7 8  1 2;
    5 6  2 2  1 3;
    3 3  8 9  1 2;
    7 7  5 6  1 3
];

%% 2. BUILD 3D OCCUPANCY GRID
blocked = false(N,N,H);

for k = 1:size(obstacles,1)
    b = obstacles(k,:);

    blocked(b(3):b(4), ...
            b(1):b(2), ...
            b(5):b(6)) = true;
end

blocked(startPoint(2),startPoint(1),startPoint(3)) = false;
blocked(goalPoint(2),goalPoint(1),goalPoint(3)) = false;

%% 3. PLAN PATH USING A*
tic;
[path,nodesExplored] = astar3D( ...
    blocked,startPoint,goalPoint);
computationTime = toc;

if isempty(path)
    error('No safe path found. Change obstacle positions.');
end

pathLength = sum(vecnorm(diff(path,1,1),2,2));

fprintf('\n===== DRONE PATH PLANNING RESULTS =====\n');
fprintf('Algorithm: 3D A*\n');
fprintf('Path points: %d\n',size(path,1));
fprintf('Path length: %.2f grid units\n',pathLength);
fprintf('Nodes explored: %d\n',nodesExplored);
fprintf('Computation time: %.6f seconds\n',computationTime);

%% 4. DRAW 3D ENVIRONMENT
fig = figure( ...
    'Name','3D Drone Path Planning', ...
    'Color',[0.04 0.07 0.12]);

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
box(ax,'on');

set(ax,'Color',[0.04 0.07 0.12], ...
    'XColor','w','YColor','w','ZColor','w');

xlim(ax,[0.5 N+0.5]);
ylim(ax,[0.5 N+0.5]);
zlim(ax,[0.5 H+0.5]);

xticks(ax,1:N);
yticks(ax,1:N);
zticks(ax,1:H);

xlabel(ax,'X Position');
ylabel(ax,'Y Position');
zlabel(ax,'Altitude Z');

title(ax,'3D Drone Path Planning and Obstacle Avoidance', ...
    'Color','w');

view(ax,42,28);
axis(ax,'vis3d');

% Draw obstacle cuboids
for k = 1:size(obstacles,1)
    b = obstacles(k,:);

    drawCuboid(ax, ...
        [b(1)-0.45,b(3)-0.45,b(5)-0.45], ...
        [b(2)+0.45,b(4)+0.45,b(6)+0.45]);
end

% Planned path
hPath = plot3(ax,path(:,1),path(:,2),path(:,3), ...
    '--','Color',[0 0.8 1],'LineWidth',2);

% Start and goal
hStart = plot3(ax,startPoint(1),startPoint(2),startPoint(3), ...
    'o','MarkerSize',10,'MarkerFaceColor',[0.1 1 0.3], ...
    'MarkerEdgeColor','w');

hGoal = plot3(ax,goalPoint(1),goalPoint(2),goalPoint(3), ...
    'p','MarkerSize',15,'MarkerFaceColor',[1 0.1 0.8], ...
    'MarkerEdgeColor','w');

% Drone marker and travelled path
hDrone = plot3(ax,startPoint(1),startPoint(2),startPoint(3), ...
    'o','MarkerSize',12,'MarkerFaceColor',[0.1 0.6 1], ...
    'MarkerEdgeColor','w');

hTrail = plot3(ax,startPoint(1),startPoint(2),startPoint(3), ...
    '-','Color',[1 0.8 0.1],'LineWidth',2);

hInfo = title(ax,'Drone ready','Color','w');

legend(ax,[hPath,hStart,hGoal,hDrone,hTrail], ...
    {'Planned path','Start','Goal','Drone','Travelled path'}, ...
    'TextColor','w','Color',[0.05 0.08 0.13], ...
    'Location','northeastoutside');

drawnow;

%% 5. SMOOTH DRONE ANIMATION
stepsPerMove = 8;
pauseTime = 0.04;
trajectory = path(1,:);

for i = 1:size(path,1)-1

    p1 = path(i,:);
    p2 = path(i+1,:);

    for j = 1:stepsPerMove

        t = j/stepsPerMove;
        pos = p1 + t*(p2-p1);

        set(hDrone,'XData',pos(1), ...
                   'YData',pos(2), ...
                   'ZData',pos(3));

        travelled = [trajectory; pos];

        set(hTrail,'XData',travelled(:,1), ...
                   'YData',travelled(:,2), ...
                   'ZData',travelled(:,3));

        set(hInfo,'String',sprintf( ...
            'Position: (%.1f, %.1f, %.1f)', ...
            pos(1),pos(2),pos(3)));

        drawnow;
        pause(pauseTime);
    end

    trajectory = [trajectory; p2];
end

set(hDrone,'XData',goalPoint(1), ...
           'YData',goalPoint(2), ...
           'ZData',goalPoint(3));

set(hInfo,'String','Destination reached!');
fprintf('Status: Destination reached successfully.\n');

%% 6. A* PATH PLANNING FUNCTION
function [path,nodesExplored] = astar3D(blocked,startP,goalP)

    [ny,nx,nz] = size(blocked);
    path = [];
    nodesExplored = 0;

    startIdx = sub2ind([ny,nx,nz], ...
        startP(2),startP(1),startP(3));

    goalIdx = sub2ind([ny,nx,nz], ...
        goalP(2),goalP(1),goalP(3));

    if blocked(startP(2),startP(1),startP(3)) || ...
       blocked(goalP(2),goalP(1),goalP(3))
        return;
    end

    gScore = inf(ny,nx,nz);
    fScore = inf(ny,nx,nz);

    parent = zeros(ny,nx,nz);
    closed = false(ny,nx,nz);

    gScore(startIdx) = 0;
    fScore(startIdx) = sum(abs(startP-goalP));

    % Six movement directions
    dirs = [
         1  0  0;
        -1  0  0;
         0  1  0;
         0 -1  0;
         0  0  1;
         0  0 -1
    ];

    while true

        candidates = find(~closed & isfinite(fScore));

        if isempty(candidates)
            return;
        end

        [~,k] = min(fScore(candidates));
        current = candidates(k);

        if current == goalIdx
            break;
        end

        closed(current) = true;
        nodesExplored = nodesExplored + 1;

        [y,x,z] = ind2sub([ny,nx,nz],current);

        for d = 1:size(dirs,1)

            xx = x + dirs(d,1);
            yy = y + dirs(d,2);
            zz = z + dirs(d,3);

            if xx < 1 || xx > nx || ...
               yy < 1 || yy > ny || ...
               zz < 1 || zz > nz
                continue;
            end

            if blocked(yy,xx,zz) || closed(yy,xx,zz)
                continue;
            end

            neighbor = sub2ind([ny,nx,nz],yy,xx,zz);
            tentativeG = gScore(current) + 1;

            if tentativeG < gScore(neighbor)

                parent(neighbor) = current;
                gScore(neighbor) = tentativeG;

                heuristic = abs(xx-goalP(1)) + ...
                            abs(yy-goalP(2)) + ...
                            abs(zz-goalP(3));

                fScore(neighbor) = tentativeG + heuristic;
            end
        end
    end

    % Reconstruct path
    current = goalIdx;
    indices = current;

    while current ~= startIdx

        current = parent(current);

        if current == 0
            path = [];
            return;
        end

        indices = [current;indices];
    end

    [ys,xs,zs] = ind2sub([ny,nx,nz],indices);
    path = [xs(:),ys(:),zs(:)];
end

%% 7. DRAW 3D OBSTACLES
function drawCuboid(ax,lo,hi)

    x = [lo(1) hi(1)];
    y = [lo(2) hi(2)];
    z = [lo(3) hi(3)];

    V = [
        x(1) y(1) z(1);
        x(2) y(1) z(1);
        x(2) y(2) z(1);
        x(1) y(2) z(1);
        x(1) y(1) z(2);
        x(2) y(1) z(2);
        x(2) y(2) z(2);
        x(1) y(2) z(2)
    ];

    F = [
        1 2 3 4;
        5 6 7 8;
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8
    ];

    patch(ax,'Vertices',V,'Faces',F, ...
        'FaceColor',[1 0.08 0.12], ...
        'FaceAlpha',0.45, ...
        'EdgeColor',[1 0.25 0.3], ...
        'LineWidth',1.2);
end
