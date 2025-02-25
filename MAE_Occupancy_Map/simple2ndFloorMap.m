clear all; close all; clc

% 3rd Floor Simplification Map
occupancyGrid = zeros(361, 114);  % Create a 50x50 grid of free space

%% Add Walls:
occupancyGrid(2:2, 80:88) = 1;
occupancyGrid(2:25, 87:88) = 1;
occupancyGrid(24:25, 87:92) = 1;
occupancyGrid(25:52, 91:92) = 1;
occupancyGrid(51:52, 91:114) = 1;
occupancyGrid(52:144, 114:114) = 1;
occupancyGrid(2:7, 79:80) = 1;
occupancyGrid(12:24, 79:80) = 1;

occupancyGrid(35:131, 35:36) = 1;
occupancyGrid(35:116, 55:56) = 1;
occupancyGrid(115:116, 35:69) = 1;
occupancyGrid(131:131, 35:75) = 1;
occupancyGrid(122:131, 68:69) = 1;
occupancyGrid(130:131, 8:75) = 1;
occupancyGrid(131:172, 8:9) = 1;
occupancyGrid(172:173, 8:80) = 1;
occupancyGrid(172:346, 79:80) = 1;
occupancyGrid(354:361, 79:80) = 1;
occupancyGrid(361:361, 80:91) = 1;
occupancyGrid(172:361, 90:91) = 1;
occupancyGrid(172:173, 90:104) = 1;
occupancyGrid(158:172, 103:104) = 1;
occupancyGrid(157:158, 90:104) = 1;
occupancyGrid(144:158, 90:91) = 1;

% % Room Rear Macs Area
occupancyGrid(23:24, 39:80) = 1;
occupancyGrid(24:35, 39:40) = 1;
occupancyGrid(24:30, 56:56) = 1;
occupancyGrid(35:36, 35:56) = 1;

% Cubicles:
occupancyGrid(62:63, 85:105) = 1;
occupancyGrid(102:103, 85:105) = 1;
occupancyGrid(116:117, 85:105) = 1;
occupancyGrid(62:106, 104:105) = 1;
occupancyGrid(114:117, 104:105) = 1;
occupancyGrid(62:117, 85:86) = 1;

% Cubicles Offices:
occupancyGrid(36:37, 62:75) = 1;
occupancyGrid(105:106, 62:75) = 1;
occupancyGrid(36:106, 62:63) = 1;
occupancyGrid(36:106, 74:75) = 1;

% Indoor Stairs
occupancyGrid(140:141, 41:79) = 1;
occupancyGrid(156:157, 41:79) = 1;
occupancyGrid(140:157, 41:42) = 1;
occupancyGrid(150:157, 78:79) = 1;
occupancyGrid(150:151, 71:79) = 1;
occupancyGrid(140:151, 65:71) = 1;

% Indoor Emergency Stairs
occupancyGrid(130:131, 90:114) = 1;
occupancyGrid(144:145, 90:114) = 1;
occupancyGrid(140:144, 90:91) = 1;
occupancyGrid(140:141, 90:103) = 1;
occupancyGrid(131:141, 96:103) = 1;

% Outdoor Emergency Stairs Corridor:
occupancyGrid(345:346, 75:80) = 1;
occupancyGrid(354:355, 75:80) = 1;
occupancyGrid(345:355, 70:75) = 1;

% Outdoor Emergency Stairs Cubicles:
occupancyGrid(6:7, 75:80) = 1;
occupancyGrid(12:13, 75:80) = 1;
occupancyGrid(6:13, 70:75) = 1;


figure
imagesc(occupancyGrid);
colormap(gray);   % Set the colormap to gray
axis equal;       % Maintain equal scaling
title('Custom Map');

% Debug: Highlight added custom wall pixels
hold on;
[yExtra, xExtra] = find(occupancyGrid == 1);
scatter(xExtra, yExtra, 10, 'red', 'filled'); % Highlight all occupied cells

%% Save the occupancy grid (Optionally)
% save(['EG-2Floor_ply', '.mat'], 'occupancyGrid');



%% Get the indoor Boundaries for defining the Monte Carlo init and final positions
% % Invert the grid so that free space is 1 and walls are 0
% freeSpaceGrid = ~occupancyGrid;
% 
% % Apply erosion to shrink the free space
% se = strel('disk', 1);  % Create a disk-shaped structuring element with radius 2
% erodedFreeSpace = imerode(freeSpaceGrid, se);
% 
% % Find all boundaries of the free space
% B = bwboundaries(erodedFreeSpace);
% 
% % Plot the result
% figure;
% subplot(1,2,1);
% imagesc(occupancyGrid);
% colormap(gray);
% axis equal;
% title('Original Occupancy Grid');
% 
% subplot(1,2,2);
% imagesc(freeSpaceGrid);
% colormap(gray);
% axis equal;
% hold on;
% 
% % Plot all boundaries
% for k = 1:length(B)
%     boundary = B{k};
%     disp(['Ploting Boundary ', num2str(k), ':']);
%     plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2);
% end
% 
% title('Free Space with All Boundaries');
% 
% % Output the coordinates of all boundaries
% disp('Boundary coordinates:');
% for k = 1:length(B)
%     disp(['Boundary ', num2str(k), ':']);
%     disp(B{k}(:, [2,1]));  % Note: we swap columns to get [x, y] format
%     disp(' ');
% end
% 
% EG_2Floor_indoor_polygon_MC_Simulation = B{2};
% EG_2Floor_Hole1_indoor_polygon_MC_Simulation = B{10};
% EG_2Floor_Hole2_indoor_polygon_MC_Simulation = B{11};
% EG_2Floor_Hole3_indoor_polygon_MC_Simulation = B{12};

%% Get the indoor Boundaries for defining the Monte Carlo init and final positions
% Invert the grid so that free space is 1 and walls are 0
freeSpaceGrid = ~occupancyGrid;

% Apply erosion to shrink the main free space
se_erode = strel('disk', 1);  % Create a disk-shaped structuring element with radius 1
erodedFreeSpace = imerode(freeSpaceGrid, se_erode);

% Find all boundaries of the free space
B_main = bwboundaries(erodedFreeSpace);

% Now, let's handle the holes separately
holeGrid = occupancyGrid;  % Create a copy for the holes

% Apply dilation to expand the holes
se_dilate = strel('disk', 2);  % Create a disk-shaped structuring element with radius 1
dilatedHoles = imdilate(holeGrid, se_dilate);

% Find the boundaries of the dilated holes
B_holes = bwboundaries(dilatedHoles);

% Plot the results
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

% Plot main boundary (number 2)
boundary_main = B_main{2};
plot(boundary_main(:,2), boundary_main(:,1), 'r', 'LineWidth', 2);

% Plot hole boundaries (10, 11, 12)
hole_indices = 2:1:5;
for i = 1:3
    disp(['Boundary Hole: ', num2str(hole_indices(i)), ':']);
    boundary_hole = B_holes{hole_indices(i)};
    plot(boundary_hole(:,2), boundary_hole(:,1), 'b', 'LineWidth', 2);
end

title('Free Space with Boundaries (Red: Main, Others: Holes)');

% Output the coordinates of the boundaries
disp('Boundary coordinates:');
disp('Main Boundary:');
disp(boundary_main(:, [2,1]));  % Note: we swap columns to get [x, y] format
disp(' ');

for i = 1:3
    disp(['Hole Boundary ', num2str(i), ':']);
    disp(B_holes{hole_indices(i)}(:, [2,1]));  % Note: we swap columns to get [x, y] format
    disp(' ');
end

% Save the boundaries
EG_2Floor_indoor_polygon_MC_Simulation = boundary_main;
EG_2Floor_Hole1_indoor_polygon_MC_Simulation = B_holes{2};
EG_2Floor_Hole2_indoor_polygon_MC_Simulation = B_holes{3};
EG_2Floor_Hole3_indoor_polygon_MC_Simulation = B_holes{4};