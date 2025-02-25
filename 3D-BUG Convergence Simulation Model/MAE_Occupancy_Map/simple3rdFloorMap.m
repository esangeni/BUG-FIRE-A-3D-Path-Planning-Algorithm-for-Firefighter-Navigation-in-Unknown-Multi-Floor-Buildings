clear all; close all; clc;

% 3rd Floor Simplification Map
% occupancyGrid = zeros(339, 75);  % Create a 50x50 grid of free space
occupancyGrid = zeros(361, 114);

%% Add Walls:
% occupancyGrid(2:132, 42:42) = 1;

occupancyGrid(2:7, 38:39) = 1;
occupancyGrid(12:132, 38:39) = 1;

occupancyGrid(2:2, 39:50) = 1;
occupancyGrid(2:132, 49:50) = 1;
occupancyGrid(132:132, 1:39) = 1;
occupancyGrid(131:132, 1:39) = 1;
occupancyGrid(131:132, 49:75) = 1;
occupancyGrid(132:173, 1:2) = 1;
occupancyGrid(132:159, 29:31) = 1;
occupancyGrid(157:159, 1:31) = 1;
occupancyGrid(132:147, 75:75) = 1;
occupancyGrid(146:147, 52:75) = 1;
occupancyGrid(146:160, 52:53) = 1;
occupancyGrid(159:160, 52:66) = 1;
occupancyGrid(160:173, 65:66) = 1;
occupancyGrid(172:174, 52:66) = 1;
occupancyGrid(172:361, 52:53) = 1;
occupancyGrid(361:361, 41:52) = 1;
occupancyGrid(354:361, 40:41) = 1;
occupancyGrid(265:346, 40:41) = 1;
occupancyGrid(265:266, 2:41) = 1;
occupancyGrid(241:265, 2:3) = 1;
occupancyGrid(240:241, 2:36) = 1;
occupancyGrid(241:254, 35:36) = 1;
occupancyGrid(253:254, 35:41) = 1;
occupancyGrid(173:254, 40:41) = 1;
occupancyGrid(173:174, 1:41) = 1;


occupancyGrid(141:145, 52:53) = 1;
occupancyGrid(141:142, 52:56) = 1;
occupancyGrid(132:142, 56:66) = 1; % Emergency stair - indoor

% Emergency stair - outdoor 
occupancyGrid(345:346, 35:41) = 1;
occupancyGrid(354:332 + 23, 35:41) = 1;
occupancyGrid(345:332 + 23, 30:35) = 1; 

% Emergency stair - outdoor MACS Corridor Area
occupancyGrid(6:7, 34:39) = 1;
occupancyGrid(12:13, 34:39) = 1;
occupancyGrid(6:13, 34:35) = 1; 

figure
imagesc(occupancyGrid);
colormap(gray);   % Set the colormap to gray
axis equal;       % Maintain equal scaling
title('Custom Map');

% % Debug: Highlight added custom wall pixels
% hold on;
% [yExtra, xExtra] = find(occupancyGrid == 1);
% scatter(xExtra, yExtra, 10, 'red', 'filled'); % Highlight all occupied cells

%% Save the occupancy grid (Optionally)
% save(['EG-3Floor_ply', '.mat'], 'occupancyGrid');


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

EG_3Floor_indoor_polygon_MC_Simulation = B{5};