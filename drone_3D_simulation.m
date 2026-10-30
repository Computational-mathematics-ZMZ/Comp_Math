clc;
clear;
close all;

%% 1. Environment
N = 10;
H = 4;

startPoint = [1 9 1];
goalPoint  = [9 1 3];

% Obstacles: [xmin xmax ymin ymax zmin zmax]
obstacles = [
    3 4 3 4 1 2
    5 6 6 7 1 3
    7 8 3 4 1 2
    2 2 7 8 1 3
    8 8 7 8 1 2
    5 6 2 2 1 3
    3 3 8 9 1 2
    7 7 5 6 1 3
];

blocked = false(N,N,H);

for k = 1:size(obstacles,1)
    o = obstacles(k,:);
    blocked(o(1):o(2),o(3):o(4),o(5):o(6)) = true;
end

blocked(startPoint(1),startPoint(2),startPoint(3)) = false;
blocked(goalPoint(1),goalPoint(2),goalPoint(3)) = false;

%% 2. Find A* path
path = astar3D(blocked,startPoint,goalPoint);

if isempty(path)
    error('No safe path found!');
end

fprintf('Path found successfully!\n');
fprintf('Path length: %d steps\n',size(path,1));

%% 3. Create 3D figure
fig = figure( ...
    'Name','Hand Gesture Drone Simulation', ...
    'NumberTitle','off', ...
    'Color',[0.08 0.10 0.15]);

ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,[0 N+1 0 N+1 0 H+1]);
axis(ax,'equal');
view(ax,42,28);

xlabel(ax,'X');
ylabel(ax,'Y');
zlabel(ax,'Altitude Z');

title(ax,'3D Drone - Hand Gesture Control','Color','w');

ax.Color = [0.08 0.10 0.15];
ax.XColor = 'w';
ax.YColor = 'w';
ax.ZColor = 'w';
ax.GridColor = [0.5 0.5 0.5];

%% 4. Draw obstacles
for k = 1:size(obstacles,1)
    o = obstacles(k,:);
    drawCuboid(ax,o,[0.85 0.15 0.15]);
end

%% 5. Draw planned path
plot3(ax,path(:,1),path(:,2),path(:,3), ...
    'b--','LineWidth',2);

plot3(ax,startPoint(1),startPoint(2),startPoint(3), ...
    'go','MarkerSize',12,'MarkerFaceColor','g');

plot3(ax,goalPoint(1),goalPoint(2),goalPoint(3), ...
    'mo','MarkerSize',12,'MarkerFaceColor','m');

text(ax,startPoint(1),startPoint(2),startPoint(3)+0.4, ...
    'START','Color','g','FontWeight','bold');

text(ax,goalPoint(1),goalPoint(2),goalPoint(3)+0.4, ...
    'GOAL','Color','m','FontWeight','bold');

%% 6. Drone graphics
position = startPoint;

drone = plot3(ax,position(1),position(2),position(3), ...
    'bo','MarkerSize',13,'MarkerFaceColor','b');

trail = plot3(ax,position(1),position(2),position(3), ...
    'c-','LineWidth',3);

trailPoints = position;

statusText = text(ax,0.5,0.5,H+1, ...
    'Command: STOP', ...
    'Color','w','FontSize',11,'FontWeight','bold');

%% 7. Speed settings
moveDelay = 0.015;
moveStep = 0.12;

% Command file used by Python
commandFile = fullfile(pwd,'command.txt');

lastCommand = 'STOP';
collisionAlarm = false;

%% 8. Main control loop
disp('Drone simulation started.');
disp('Start Python hand_detection.py to control the drone.');

while ishandle(fig)

    % Read Python command
    command = 'STOP';

    if isfile(commandFile)
        try
            command = upper(strtrim(fileread(commandFile)));
        catch
            command = 'STOP';
        end
    end

    % Calculate movement
    newPosition = position;

    switch command
        case 'FORWARD'
            newPosition(2) = newPosition(2)+moveStep;

        case 'BACKWARD'
            newPosition(2) = newPosition(2)-moveStep;

        case 'RIGHT'
            newPosition(1) = newPosition(1)+moveStep;

        case 'LEFT'
            newPosition(1) = newPosition(1)-moveStep;

        case 'UP'
            newPosition(3) = newPosition(3)+moveStep;

        case 'DOWN'
            newPosition(3) = newPosition(3)-moveStep;

        otherwise
            newPosition = position;
    end

    % Keep drone inside the environment
    newPosition = max(newPosition,[1 1 1]);
    newPosition = min(newPosition,[N N H]);

    % Check collision
    collision = checkCollision(blocked,newPosition);

    if collision
        if ~collisionAlarm
            beep;
            fprintf('WARNING: Drone collision detected!\n');
            collisionAlarm = true;
        end

        newPosition = position;

        set(statusText,'String', ...
            ['COLLISION! Command: ' command], ...
            'Color','r');

    else
        collisionAlarm = false;

        set(statusText,'String', ...
            ['Command: ' command], ...
            'Color','w');
    end

    % Update drone only when movement is safe
    if any(newPosition ~= position)

        position = newPosition;

        set(drone, ...
            'XData',position(1), ...
            'YData',position(2), ...
            'ZData',position(3));

        trailPoints = [trailPoints; position];

        set(trail, ...
            'XData',trailPoints(:,1), ...
            'YData',trailPoints(:,2), ...
            'ZData',trailPoints(:,3));

    end

    drawnow limitrate;
    pause(moveDelay);

end

disp('Simulation stopped.');

%% 9. Collision detection function
function collision = checkCollision(blocked,p)

    x = round(p(1));
    y = round(p(2));
    z = round(p(3));

    collision = false;

    if x < 1 || x > size(blocked,1) || ...
       y < 1 || y > size(blocked,2) || ...
       z < 1 || z > size(blocked,3)

        collision = true;
        return;
    end

    collision = blocked(x,y,z);

end

%% 10. Draw cuboid function
function drawCuboid(ax,o,color)

    x1=o(1)-0.5; x2=o(2)+0.5;
    y1=o(3)-0.5; y2=o(4)+0.5;
    z1=o(5)-0.5; z2=o(6)+0.5;

    vertices = [
        x1 y1 z1
        x2 y1 z1
        x2 y2 z1
        x1 y2 z1
        x1 y1 z2
        x2 y1 z2
        x2 y2 z2
        x1 y2 z2
    ];

    faces = [
        1 2 3 4
        5 6 7 8
        1 2 6 5
        2 3 7 6
        3 4 8 7
        4 1 5 8
    ];

    patch(ax, ...
        'Vertices',vertices, ...
        'Faces',faces, ...
        'FaceColor',color, ...
        'FaceAlpha',0.85, ...
        'EdgeColor','w');

end

%% 11. A* 3D path planning
function path = astar3D(blocked,startPoint,goalPoint)

    moves = [
         1  0  0
        -1  0  0
         0  1  0
         0 -1  0
         0  0  1
         0  0 -1
    ];

    dims = size(blocked);

    openSet = startPoint;
    gScore = inf(dims);
    fScore = inf(dims);

    parent = zeros([dims 3]);

    gScore(startPoint(1),startPoint(2),startPoint(3)) = 0;

    fScore(startPoint(1),startPoint(2),startPoint(3)) = ...
        norm(startPoint-goalPoint);

    path = [];

    while ~isempty(openSet)

        scores = zeros(size(openSet,1),1);

        for i = 1:size(openSet,1)
            p = openSet(i,:);
            scores(i) = fScore(p(1),p(2),p(3));
        end

        [~,idx] = min(scores);
        current = openSet(idx,:);

        openSet(idx,:) = [];

        if isequal(current,goalPoint)

            path = current;

            while ~isequal(current,startPoint)

                p = squeeze(parent( ...
                    current(1),current(2),current(3),:))';

                if all(p==0)
                    path = [];
                    return;
                end

                current = p;
                path = [current;path];

            end

            return;
        end

        for i = 1:size(moves,1)

            neighbor = current + moves(i,:);

            if any(neighbor < 1) || ...
               any(neighbor > dims)
                continue;
            end

            x=neighbor(1);
            y=neighbor(2);
            z=neighbor(3);

            if blocked(x,y,z)
                continue;
            end

            tentativeG = ...
                gScore(current(1),current(2),current(3))+1;

            if tentativeG < gScore(x,y,z)

                parent(x,y,z,:) = current;

                gScore(x,y,z) = tentativeG;

                fScore(x,y,z) = tentativeG + ...
                    norm(neighbor-goalPoint);

                if ~ismember(neighbor,openSet,'rows')
                    openSet = [openSet;neighbor];
                end

            end
        end
    end

end
