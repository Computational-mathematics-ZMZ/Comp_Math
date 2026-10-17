function distance = calculatePathLength(path)
% Calculate path length as number of movements.
% Each row of path represents [row, column].

if isempty(path)
    distance = 0;
else
    distance = size(path, 1) - 1;
end

end