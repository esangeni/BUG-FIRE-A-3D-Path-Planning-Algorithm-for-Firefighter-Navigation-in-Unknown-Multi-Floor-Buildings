clear all; close all; clc;
% 4th Floor Simplification Map
% occupancyGrid = zeros(55,76);  % Create a 50x50 grid of free space
% occupancyGrid = zeros(339, 76);  % Create a 50x50 grid of free space
occupancyGrid = zeros(361, 114);

%% Add Walls:
occupancyGrid(122:132, 44:45) = 1;
occupancyGrid(145:159, 52:53) = 1;
occupancyGrid(158:159, 52:67) = 1;
occupancyGrid(159:171, 66:67) = 1;
occupancyGrid(171:175, 52:67) = 1;
occupancyGrid(171:175, 4:41) = 1;

occupancyGrid(171:171, 4:41) = 1;
occupancyGrid(133:171, 4:4) = 1;
occupancyGrid(133:133, 4:32) = 1;
occupancyGrid(2+120:133, 4:32) = 1;

occupancyGrid(133:158, 29:30) = 1;
occupancyGrid(157:158, 4:30) = 1;

% Emergency stairs - Indoor
occupancyGrid(132:133, 44:76) = 1;
occupancyGrid(143:144, 52:76) = 1;
occupancyGrid(139:144, 52:53) = 1;
occupancyGrid(139:140, 52:57) = 1;
occupancyGrid(133:144, 76:76) = 1;
occupancyGrid(133:140, 58:63) = 1; 

% Hallway 1
occupancyGrid(172:361, 52:53) = 1;
occupancyGrid(361:361, 41:52) = 1;
occupancyGrid(172:346, 40:41) = 1;
occupancyGrid(354:361, 40:41) = 1;

% Emergency Stairs Hallway 1
occupancyGrid(346:354, 30:37) = 1;
occupancyGrid(345:346, 30:41) = 1;
occupancyGrid(354:355, 30:41) = 1;

% Hallway 2
occupancyGrid(1:2, 41:52) = 1;
occupancyGrid(1:7, 40:41) = 1;
occupancyGrid(1:122, 52:53) = 1;
occupancyGrid(122:123, 44:53) = 1;
occupancyGrid(113:125, 31:32) = 1;
occupancyGrid(112:113, 31:41) = 1;
occupancyGrid(12:113, 40:41) = 1;

% Emergency Stairs Hallway 2
occupancyGrid(6:7, 31:41) = 1;
occupancyGrid(12:13, 31:41) = 1;
occupancyGrid(6:13, 31:37) = 1;


figure
imagesc(occupancyGrid);
colormap(gray);   % Set the colormap to gray
axis equal;       % Maintain equal scaling
title('Custom Map');

% Debug: Highlight added custom wall pixels
% hold on;
% [yExtra, xExtra] = find(occupancyGrid == 1);
% scatter(xExtra, yExtra, 10, 'red', 'filled'); % Highlight all occupied cells

%% Save the occupancy grid (Optionally)
save(['EG-4Floor_ply', '.mat'], 'occupancyGrid');

%% Get the indoor Boundaries for defining the Monte Carlo init and final positions
% Invert the grid so that free space is 1 and walls are 0
freeSpaceGrid = ~occupancyGrid;

% Apply erosion to shrink the free space
se = strel('disk', 1);  % Create a disk-shaped structuring element with radius 2
erodedFreeSpace = imerode(freeSpaceGrid, se);

% Find all boundaries of the free space
B = bwboundaries(erodedFreeSpace);

% Plot the result
figure;
subplot(1,2,1);
imagesc(occupancyGrid);
colormap(gray);
axis equal;
title('Original Occupancy Grid');

subplot(1,2,2);
imagesc(freeSpaceGrid);
colormap(gray);
axis equal;
hold on;

% Plot all boundaries
for k = 1:length(B)
    boundary = B{k};
    disp(['Ploting Boundary ', num2str(k), ':']);
    plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);
end

title('Free Space with All Boundaries');

% Output the coordinates of all boundaries
disp('Boundary coordinates:');
for k = 1:length(B)
    disp(['Boundary ', num2str(k), ':']);
    disp(B{k}(:, [2,1]));  % Note: we swap columns to get [x, y] format
    disp(' ');
end

EG_4Floor_indoor_polygon_MC_Simulation = B{3};