clear all; close all; clc; warning off;
%% File Header
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Description:
%   This script implements a Monte Carlo Simulation to evaluate the 
%   performance of Eudald's BUG3 algorithm in a 3D environment, specifically
%   the Engineering Gateway building at the University of California, Irvine.
%
%   The simulation aims to:
%   1. Generate random start and goal positions across multiple floors.
%   2. Execute the BUG3 algorithm for each scenario.
%   3. Analyze the success rate, path lengths, and execution times.
%   4. Demonstrate the algorithm's ability to navigate in a complex 3D space
%      without prior knowledge of the complete map.
%
%   Key Features:
%   - 3D path planning across multiple floors
%   - Handling of obstacles and emergency exits
%   - Performance analysis through multiple iterations
%   - Visualization of the building layout and paths
%
%   Author: Eudald Sangenis Rafart
%   Affiliation: University of California, Irvine
%   Email: esangeni@uci.edu
%   Date: 18 Feb. 2025
%   Last Revision: [Current Date]
%
%   Copyright (c) 2025, Eudald Sangenis Rafart
%   All rights reserved.
%
%   Algorithm used:
%
%   1. Eudald's BUG3
%
%   Notes:
%   - The script uses a modified version of the Navigation function from
%     Peter Corke's Robotics Toolbox.
%       * In Navigation function from "C:\Users\Eudald\Documents\GitRepos\PeterCork_PathPlanning_Lib\rtb",
%         I commented this line of code (544): "assert(~nav.isoccupied(nav.goal(1:2)), 'Navigation:checkquery:badarg', 'goal location inside obstacle');"
%         Cause when checking for the feasible goal position is on the same 
%         floor not at a different one.
%   However in this code:
%   - A navigation mesh is implemented to determine feasible start and goal
%     positions across different floors.
%   - The simulation includes timeout mechanisms and error handling for
%     robust performance evaluation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start of the code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Libraries
% Add necessary paths for external libraries and custom functions
currentFolder = pwd;
addpath(genpath('C:\Users\Eudald\Documents\Git Repos\PeterCork_PathPlanning_Lib\rtb'));
addpath(genpath('C:\Users\Eudald\Documents\Git Repos\PeterCork_PathPlanning_Lib\common'));
addpath(genpath('C:\Users\Eudald\Documents\Git Repos\PeterCork_PathPlanning_Lib\smtb'));
addpath(genpath([currentFolder, '\MAE_Occupancy_Map']));
addpath(genpath([currentFolder, '\lib\Plots']));
addpath(genpath([currentFolder, '\lib\Navigation']));

% Get the screen size and calculate dimensions for the figure
screenSize = get(0, 'ScreenSize');
width = round(screenSize(3) * 0.9);
height = round(screenSize(4) * 0.8);
left = round((screenSize(3) - width) / 2);
bottom = round((screenSize(4) - height) / 2);

%% Monte Carlo Simulation Parameters
useParallel = true; % Flag to enable/disable parallel execution
visionRadius = 30; % Agent's vision radius in meters
visionAngle = 170; % Agent's vision angle in degrees
numSimulations = 250; % Number of simulations to run
randomPointsGeneration = false; % Flag to generate random points or use pre-saved points

% Initialize counters and arrays for results
successCount = 0;
failureCount = 0;
pathLengths = [];
executionTimes = [];

% Create a table to store detailed results of each simulation
resultsTable = table('Size', [numSimulations 6], ...
                     'VariableTypes', {'double', 'string', 'string', 'logical', 'double', 'double'}, ...
                     'VariableNames', {'Iteration', 'StartPoint', 'GoalPoint', 'GoalReached', 'ExecutionTime', 'PathLength'});

%% Load polygon data for the building floors
% Load pre-defined polygon data for each floor and holes
load('MAE_Occupancy_Map/EG-2Floor_indoor_polygon_MC_Simulation.mat');
load('MAE_Occupancy_Map/EG-2Floor_Hole1_indoor_polygon_MC_Simulation.mat');
load('MAE_Occupancy_Map/EG-2Floor_Hole2_indoor_polygon_MC_Simulation.mat');
load('MAE_Occupancy_Map/EG-2Floor_Hole3_indoor_polygon_MC_Simulation.mat');
load('MAE_Occupancy_Map/EG-3Floor_indoor_polygon_MC_Simulation.mat');
load('MAE_Occupancy_Map/EG-4Floor_indoor_polygon_MC_Simulation.mat');

% Swap x and y coordinates and apply necessary translations
secondFloorPolygon = EG_2Floor_indoor_polygon_MC_Simulation(:, [2, 1]);
holes = {
    EG_2Floor_Hole1_indoor_polygon_MC_Simulation(:, [2, 1]),
    EG_2Floor_Hole2_indoor_polygon_MC_Simulation(:, [2, 1]),
    EG_2Floor_Hole3_indoor_polygon_MC_Simulation(:, [2, 1])
};

thirdFloorPolygon = EG_3Floor_indoor_polygon_MC_Simulation(:, [2, 1]);
thirdFloorPolygon(:, 1) = thirdFloorPolygon(:, 1) + 39;  % Translate x-coordinates

fourthFloorPolygon = EG_4Floor_indoor_polygon_MC_Simulation(:, [2, 1]);
fourthFloorPolygon(:, 1) = fourthFloorPolygon(:, 1) + 37;  % Translate x-coordinates

% Combine polygons into a cell array
polygons = {
    {0, secondFloorPolygon, holes},
    {100, thirdFloorPolygon},
    {200, fourthFloorPolygon}
};

%% Display polygon areas (visualization of the building layout)
if 0
    % Generate and display the 3D map of Engineering Gateway
    maps = generateEGMap3DBUG();
    
    figureHandle = findobj('Type', 'Figure', 'Name', '3D Trajectory MAE UCI');
    set(figureHandle, 'Position', [left, bottom, width, height]);
    if isempty(figureHandle)
        error('The figure generated by generateEGMap3DBUG does not exist. Ensure the function has executed correctly.');
    end
    
    hold on;
    
    % Plot each floor with a different shade of red
    colors = [1 0.4 0.4; 1 0.6 0.6; 1 0.8 0.8];
    
    for i = 1:length(polygons)
        floorHeight = polygons{i}{1};
        polygon = polygons{i}{2};
        
        z = ones(size(polygon, 1), 1) * (floorHeight);
        
        % Plot floor area and outline
        fill3(polygon(:,1), polygon(:,2), z, colors(i,:), 'FaceAlpha', 0.5);
        plot3(polygon(:,1), polygon(:,2), z, 'k-', 'LineWidth', 2);
        
        % Plot holes for 2nd floor
        if i == 1
            for j = 1:length(holes)
                hole = holes{j};
                z_hole = zeros(size(hole, 1), 1) + floorHeight;
                fill3(hole(:,1), hole(:,2), z_hole+0.1, 'w-', 'LineWidth', 3);
            end
        end
    end

    % Set up the plot aesthetics
    xlabel('$X,\, \mathrm{m}$', 'Interpreter', 'latex');
    ylabel('$Y,\, \mathrm{m}$', 'Interpreter', 'latex');
    title('Monte Carlo Simulation Areas', 'Interpreter', 'latex');
    axis equal;
    grid on;
    legend('2nd Floor', '3rd Floor', '4th Floor', '2nd Floor Holes');
    
    % Convert pixel ticks to meter values
    ax = gca;
    xticks_pixels = ax.XTick;
    xticks_meters = xticks_pixels * 0.25;
    xticks(xticks_pixels);
    xticklabels(arrayfun(@(x) sprintf('%.1f', x), xticks_meters, 'UniformOutput', false));
    yticks_pixels = ax.YTick;
    yticks_meters = yticks_pixels * 0.25;
    yticks(yticks_pixels);
    yticklabels(arrayfun(@(x) sprintf('%.1f', x), yticks_meters, 'UniformOutput', false));
    
    hold off;
end

%% Run Monte Carlo Simulation
h = waitbar(0, 'Starting Monte Carlo Simulation...', 'Name', 'Progress');
timeoutDuration = 120; % 2 minutes timeout

% Start a parallel pool if not already running and if parallel execution is
% enabled. Parallel pool will not display route plots!
if useParallel && isempty(gcp('nocreate'))
    parpool('local');
end

for i = 1:numSimulations
    % Calculate and display current success rate to update the waitbar
    if i > 1
        currentSuccessRate = (successCount / (i-1)) * 100;
    else
        currentSuccessRate = 0;
    end
    waitbar(i/numSimulations, h, sprintf('Running simulation %d of %d\nCurrent Success Rate: %.2f%%', i, numSimulations, currentSuccessRate));
    
    % Generate or load start and goal positions
    if randomPointsGeneration
        [startPoint, goalPoint] = generateRandomPositions(polygons);
    else
        % Read pre-generated points from an Excel file
        filename = '250iterResults_v170_d30_2025-02-27_18-32-01.xlsx';
        fullPath = fullfile(pwd, filename);
        
        if ~exist(fullPath, 'file')
            error('File not found: %s', fullPath);
        end
        
        data = readtable(fullPath);
        startPointStr = data.StartPoint{i};
        goalPointStr = data.GoalPoint{i};
        startPoint = str2num(startPointStr(2:end-1));
        goalPoint = str2num(goalPointStr(2:end-1));
    end

    startPoint = round(startPoint);
    goalPoint = round(goalPoint);

    disp("Origin Position: " + num2str(startPoint(1)) + ", " + num2str(startPoint(2)) + ", " + num2str(startPoint(3)))
    disp("Goal Position: " + num2str(goalPoint(1)) + ", " + num2str(goalPoint(2)) + ", " + num2str(goalPoint(3)))
    
    % Run the Bug3 algorithm with timeout
    tic;
    if useParallel
        % Use parallel execution
        f = parfeval(@eudaldSangenis_BUG3_fcn, 3, startPoint, goalPoint, visionRadius, visionAngle);
        try
            [completedIdx, path, trajectory_length, goal3DReached] = fetchNext(f, timeoutDuration);
            executionTime = toc;
            
            if completedIdx > 0
                if goal3DReached
                    successCount = successCount + 1;
                    pathLengths(end+1) = trajectory_length; % size(path, 1)
                    executionTimes(end+1) = executionTime;
                    disp('Path found successfully!');
                else
                    failureCount = failureCount + 1;
                    disp('Failed to find a path.');
                end
            else
                % Timeout occurred
                cancel(f);
                failureCount = failureCount + 1;
                goal3DReached = false;
                executionTime = timeoutDuration;
                disp('Simulation timed out after 2 minutes.');
            end
        catch ME
            cancel(f);
            executionTime = toc;
            failureCount = failureCount + 1;
            goal3DReached = false;
            disp('Error occurred during simulation.');
            disp(ME.message);
            disp(ME.stack(1));
        end
    else
        % Run without parallel execution
        try
            [path, trajectory_length,goal3DReached] = eudaldSangenis_BUG3_fcn(startPoint, goalPoint, visionRadius, visionAngle);
            executionTime = toc;
            
            if goal3DReached
                successCount = successCount + 1;
                pathLengths(end+1) = trajectory_length; % size(path, 1)
                executionTimes(end+1) = executionTime;
                disp('Path found successfully!');
            else
                failureCount = failureCount + 1;
                disp('Failed to find a path.');
            end
        catch ME
            executionTime = toc;
            failureCount = failureCount + 1;
            goal3DReached = false;
            disp('Error occurred during simulation.');
            disp(ME.message);
        end
        
        % Check for timeout
        if executionTime >= timeoutDuration
            disp('Simulation timed out after 2 minutes.');
            goal3DReached = false;
            executionTime = timeoutDuration;
        end
    end

    if ~exist('trajectory_length', 'var')
        trajectory_length = NaN;
    end

    % Add results to the table
    resultsTable.Iteration(i) = i;
    resultsTable.StartPoint(i) = sprintf('[%d, %d, %d]', startPoint(1), startPoint(2), startPoint(3));
    resultsTable.GoalPoint(i) = sprintf('[%d, %d, %d]', goalPoint(1), goalPoint(2), goalPoint(3));
    resultsTable.GoalReached(i) = goal3DReached;
    resultsTable.ExecutionTime(i) = executionTime;
    resultsTable.PathLength(i) = trajectory_length; % Convert pixels to meters - size(path, 1) * 0.25

    fprintf('Simulation %d/%d completed. Success: %d, Failure: %d, Current Success Rate: %.2f%%\n', ...
    i, numSimulations, successCount, failureCount, (successCount / i) * 100);
end

% Close the waitbar
close(h);

%% Analyze and Display Results
successRate = successCount / numSimulations * 100;
averagePathLength = mean(resultsTable.PathLength);
averageExecutionTime = mean(resultsTable.ExecutionTime);

disp('------------------------------------------------------------------------');
disp('Monte Carlo Simulation Results:');
disp(['Number of simulations: ', num2str(numSimulations)]);
disp(['Success rate: ', num2str(successRate), '%']);
disp(['Average path length: ', num2str(averagePathLength), ' [m]']);
disp(['Average execution time: ', num2str(averageExecutionTime), ' seconds']);

% Display the results table
disp('------------------------------------------------------------------------');
disp('Detailed Results:');
disp(resultsTable);

% Save results to an Excel file
current_datetime = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss'); 
filename = [num2str(numSimulations) 'iterResults_v' num2str(visionAngle) '_d' num2str(visionRadius) '_' char(current_datetime) '.xlsx'];
writetable(resultsTable, filename, 'Sheet', 1);

%% Helper Functions
% - generatePointInPolygon: Generates a random point within a given polygon
% - isPointAwayFromEdges: Checks if a point is sufficiently far from polygon edges
% - pointToLineDistance: Calculates the distance from a point to a line segment
% - generateRandomPositions: Generates random start and goal positions for the simulation

function point = generatePointInPolygon(floorHeight, polygon, holes)
    minX = min(polygon(:,1));
    maxX = max(polygon(:,1));
    minY = min(polygon(:,2));
    maxY = max(polygon(:,2));
    
    % Define a minimum distance from the edges (adjust as needed)
    minDistanceFromEdge = 1.5; % 1 unit away from edges
    
    while true
        x = minX + rand() * (maxX - minX);
        y = minY + rand() * (maxY - minY);
        
        if inpolygon(x, y, polygon(:,1), polygon(:,2))
            % Check if the point is far enough from all edges
            if isPointAwayFromEdges(x, y, polygon, minDistanceFromEdge)
                isInHole = false;
                for i = 1:length(holes)
                    if inpolygon(x, y, holes{i}(:,1), holes{i}(:,2))
                        isInHole = true;
                        break;
                    end
                end
                if ~isInHole
                    point = [x, y, floorHeight];
                    break;
                end
            end
        end
    end
end

function isAway = isPointAwayFromEdges(x, y, polygon, minDistance)
    isAway = true;
    n = size(polygon, 1);
    for i = 1:n
        j = mod(i, n) + 1; % Next point (wrapping around to 1 for the last point)
        dist = pointToLineDistance(x, y, polygon(i,1), polygon(i,2), polygon(j,1), polygon(j,2));
        if dist < minDistance
            isAway = false;
            break;
        end
    end
end

function d = pointToLineDistance(x, y, x1, y1, x2, y2)
    % Distance from point (x,y) to line segment from (x1,y1) to (x2,y2)
    A = x - x1;
    B = y - y1;
    C = x2 - x1;
    D = y2 - y1;

    dot = A * C + B * D;
    len_sq = C * C + D * D;
    param = dot / len_sq;

    if param < 0
        xx = x1;
        yy = y1;
    elseif param > 1
        xx = x2;
        yy = y2;
    else
        xx = x1 + param * C;
        yy = y1 + param * D;
    end

    dx = x - xx;
    dy = y - yy;
    d = sqrt(dx*dx + dy*dy);
end

function [startPoint, goalPoint] = generateRandomPositions(polygons)
    % Generate start point
    startFloorIndex = randi([1, length(polygons)]);
    startFloorHeight = polygons{startFloorIndex}{1};
    startPolygon = polygons{startFloorIndex}{2};
    startHoles = {};
    if length(polygons{startFloorIndex}) > 2
        startHoles = polygons{startFloorIndex}{3};
    end
    startPoint = generatePointInPolygon(startFloorHeight, startPolygon, startHoles);
    
    % Generate goal point (ensure it's different from start point)
    while true
        goalFloorIndex = randi([1, length(polygons)]);
        goalFloorHeight = polygons{goalFloorIndex}{1};
        goalPolygon = polygons{goalFloorIndex}{2};
        goalHoles = {};
        if length(polygons{goalFloorIndex}) > 2
            goalHoles = polygons{goalFloorIndex}{3};
        end
        goalPoint = generatePointInPolygon(goalFloorHeight, goalPolygon, goalHoles);
        
        if ~isequal(startPoint, goalPoint)
            break;
        end
    end
end