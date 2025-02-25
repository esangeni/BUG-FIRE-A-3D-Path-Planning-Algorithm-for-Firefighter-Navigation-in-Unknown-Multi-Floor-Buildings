clear all; close all; clc;

%% Select Point Cloud
% EG-1stFloor_ply.ply, EG-2ndFloor_ply.ply, EG-3rdFloor_ply.ply, EG-4thFloor_ply.ply
mapName = 'EG-2ndFloor_ply.ply';

%% Load point cloud
ptCloud = pcread(mapName);

pcshow(ptCloud)

hold on
plot3([x_limits(1) x_limits(2) x_limits(2) x_limits(1) x_limits(1) x_limits(1) x_limits(2) x_limits(2)], ...
      [y_limits(1) y_limits(1) y_limits(2) y_limits(2) y_limits(1) y_limits(1) y_limits(1) y_limits(2)], ...
      [z_limits(1) z_limits(1) z_limits(1) z_limits(1) z_limits(1) z_limits(2) z_limits(2) z_limits(2)], 'r-')
hold off

dimensions = [diff(x_limits), diff(y_limits), diff(z_limits)];
title(sprintf('Point Cloud: %s\nDimensions: %.2f x %.2f x %.2f', mapName, dimensions(1), dimensions(2), dimensions(3)))

% Add axis labels and title for better visualization
xlabel('X');
ylabel('Y');
zlabel('Z');
title(['Point Cloud: ', mapName]);

% Get the limits of the point cloud using the Location property
x_limits = [min(ptCloud.Location(:, 1)), max(ptCloud.Location(:, 1))];
y_limits = [min(ptCloud.Location(:, 2)), max(ptCloud.Location(:, 2))];
z_limits = [min(ptCloud.Location(:, 3)), max(ptCloud.Location(:, 3))];

% Set the axis limits to encompass the entire point cloud
axis([x_limits, y_limits, z_limits]);
set(gca, 'Color', [0.9 0.9 0.9]);
% Add a grid for better understanding of the dimensions
grid on;

fprintf('X range: %.2f to %.2f\n', x_limits(1), x_limits(2))
fprintf('Y range: %.2f to %.2f\n', y_limits(1), y_limits(2))
fprintf('Z range: %.2f to %.2f\n', z_limits(1), z_limits(2))

%% Extract points
points = ptCloud.Location; % Get [x, y, z] points

%% Filter for walls
if mapName == "EG-2ndFloor_ply.ply"
    wallPoints = points(points(:, 3) > 5 & points(:, 3) < 5.6, :); % 4th Floor Limits

elseif mapName == "EG-3rdFloor_ply.ply"
    wallPoints = points(points(:, 3) > 9.5 & points(:, 3) < 10.75 & points(:, 2) > -27.4, :); % 4th Floor Limits

elseif mapName == "EG-4thFloor_ply.ply"
    wallPoints = points(points(:, 3) > 14 & points(:, 3) < 15, :); % 4th Floor Limits
end

% Project onto 2D plane (x, y)
wallPoints2D = wallPoints(:, 1:2);

% Define grid resolution
gridResolution = 0.25; % Resolution of the grid (meters per cell)

% Find the limits of the grid
xLimits = [min(wallPoints2D(:, 1)), max(wallPoints2D(:, 1))];
yLimits = [min(wallPoints2D(:, 2)), max(wallPoints2D(:, 2))];

%% Create occupancy grid
xGrid = ceil((wallPoints2D(:, 1) - xLimits(1)) / gridResolution) + 1;
yGrid = ceil((wallPoints2D(:, 2) - yLimits(1)) / gridResolution) + 1;

% Initialize grid matrix
gridSizeX = max(xGrid); % Use max index for grid size
gridSizeY = max(yGrid);
occupancyGrid = zeros(gridSizeY, gridSizeX);

% Mark wall points in the grid
for i = 1:length(xGrid)
    occupancyGrid(yGrid(i), xGrid(i)) = 1;
end

%% Visualize the occupancy grid
figure;
imagesc(occupancyGrid);
colormap(gray); % Walls are black, free space is white
axis equal;
title('2D Occupancy Grid Map from 3D Point Cloud');
xlabel('X (grid cells)');
ylabel('Y (grid cells)');

% % Debug: Highlight added custom wall pixels
% hold on;
% [yExtra, xExtra] = find(occupancyGrid == 1);
% scatter(xExtra, yExtra, 10, 'red', 'filled'); % Highlight all occupied cells

%% Save the occupancy grid (Optionally)
% save([mapName, '.mat'], 'occupancyGrid');
