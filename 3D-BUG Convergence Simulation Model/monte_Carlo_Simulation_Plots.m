clear all; close all; clc;
set(0, 'defaultTextInterpreter', 'latex');
set(0, 'defaultAxesTickLabelInterpreter', 'latex');
set(0, 'defaultLegendInterpreter', 'latex');


% Import all the datasets with the performance of the BUG3 algorithm with
% different radius parameters
% t_r30 = readtable('250iterResults_v170_d30_2025-02-27_18-32-01.xlsx');
% t_r10 = readtable('250iterResults_v170_d10_2025-02-27_20-00-48.xlsx');
% t_r5 = readtable('250iterResults_v170_d5_2025-02-27_22-03-30.xlsx');
% t_r1 = readtable('250iterResults_v170_d1_2025-02-28_00-07-03.xlsx');
% t_r05 = readtable('250iterResults_v170_d0.5_2025-02-28_10-24-18.xlsx');
t_r30 = readtable('250iterResults_v170_d30_2025-02-28_23-17-16.xlsx');
t_r10 = readtable('250iterResults_v170_d10_2025-02-28_20-36-56.xlsx');
t_r5 = readtable('250iterResults_v170_d5_2025-02-28_19-13-17.xlsx');
t_r1 = readtable('250iterResults_v170_d1_2025-02-28_15-47-13.xlsx');
t_r05 = readtable('250iterResults_v170_d0.5_2025-02-28_14-15-31.xlsx');


% Initialize counters for each floor combination
floor_combinations = {...
    '2nd-2nd', '3rd-3rd', '4th-4th', ...
    '2nd-3rd', '2nd-4th', ...
    '3rd-2nd', '3rd-4th', ...
    '4th-2nd', '4th-3rd'...
};
counts = zeros(1, length(floor_combinations));

% Bar plot - Number Simulations Per Floor Combination
plotNumberSimulationsPerFloorCombination(t_r30, floor_combinations, counts)
% 3D bar plot - Average Trajectory Length Per Floor Combination and Radius
plot3DBarAverageTrajectoryLength(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations);

%% Helper functions:
function plotNumberSimulationsPerFloorCombination(t, floor_combinations, counts)
    % Count occurrences of each floor combination
    for i = 1:size(t, 1)
        % Parse start point
        startStr = t.StartPoint{i};
        startPoint = str2num(startStr(2:end-1));
        
        % Parse goal point
        goalStr = t.GoalPoint{i};
        goalPoint = str2num(goalStr(2:end-1));
        
        % Determine start and goal floors
        startFloor = startPoint(3)/100 + 2; % (2nd, 3rd, 4th floor)
        goalFloor = goalPoint(3)/100 + 2; % (2nd, 3rd, 4th floor)
        
        % Increment appropriate counter
        if startFloor == goalFloor % cases: '2nd-2nd', '3rd-3rd', '4th-4th'
            counts(startFloor - 1) = counts(startFloor - 1) + 1;
        else
            if startFloor == 2 
                if goalFloor == 3
                    counts(4) = counts(4) + 1;
                elseif goalFloor == 4
                    counts(5) = counts(5) + 1;
                end
            elseif startFloor == 3
                if goalFloor == 2
                    counts(6) = counts(6) + 1;
                elseif goalFloor == 4
                    counts(7) = counts(7) + 1;
                end
            elseif startFloor == 4
                if goalFloor == 2
                    counts(8) = counts(8) + 1;
                elseif goalFloor == 3
                    counts(9) = counts(9) + 1;
                end
            end
        end
    end
    
    % Create bar plot
    figure;
    bar(counts);
    xlabel('Floor Combinations');
    ylabel('Number of Routes');
    title('Distribution of Routes Across Floor Combinations');
    xticks(1:length(floor_combinations));
    xticklabels(floor_combinations);
    xtickangle(45);
    
    % Add value labels on top of each bar
    for i = 1:length(counts)
        text(i, counts(i), num2str(counts(i)), 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
    
    % Calculate and display percentages
    total_routes = sum(counts);
    percentages = (counts / total_routes) * 100;
    
    disp('Distribution of routes:');
    for i = 1:length(floor_combinations)
        fprintf('%s: %d (%.2f%%)\n', floor_combinations{i}, counts(i), percentages(i));
    end
    
    % Display total number of routes
    fprintf('\nTotal number of routes: %d\n', total_routes);
end

function plot3DBarAverageTrajectoryLength(t_r05, t_r1, t_r5, t_r10, t_r30, floor_combinations)
    % Define radii
    radii = [0.5, 1, 5, 10, 30];
    
    % Initialize matrix to store average lengths
    avg_lengths = zeros(length(radii), length(floor_combinations));
    
    % Process data for each radius
    tables = {t_r05, t_r1, t_r5, t_r10, t_r30};
    for r = 1:length(radii)
        t = tables{r};
        avg_lengths(r, :) = calculateAverageLengths(t, floor_combinations);
    end
    
    % Create 3D bar plot
    figure;
    h = bar3(avg_lengths);    
    
    % Customize the plot
    xlabel('Floor Combinations','FontSize',18);
    ylabel('Vision Radius (m)','FontSize',18);
    zlabel('Average Trajectory Length (m)','FontSize',18);
    title('Average Trajectory Length by Floor Combination and Vision Radius','FontSize',18);
    
    % Set x-axis labels
    xticklabels(floor_combinations);
    xtickangle(45);
    
    % Set y-axis labels
    yticks(1:length(radii));
    yticklabels(radii);

    % Set font size for tick labels
    ax = gca;
    ax.XAxis.FontSize = 18;
    ax.YAxis.FontSize = 18;
    ax.ZAxis.FontSize = 18;
    
    % Adjust view for better visibility
    view(19.5,25.7);

    % Display numeric values on top of each bar
    for i = 1:size(avg_lengths, 1)
        for j = 1:size(avg_lengths, 2)
            % Determine text color based on floor combination
            if ismember(j, [1, 2, 8])  % 2nd-2nd, 3rd-3rd, and 4th-2nd
                textColor = 'white';
            else
                textColor = 'black';
            end
            
            text(j, i, avg_lengths(i,j), num2str(avg_lengths(i,j), '%.1f'), ...
                 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
                 'FontSize', 18, 'Color', textColor);
        end
    end

    % Face Colors:
    f_colors = [0 0 0;
                .5 .5 .5;
                .9 .9 .9;
                .85 .33 .1;
                .93 .69 .13;
                0.3647, 0.8000, 0.4549; % 93, 204, 116;
                0.59, 1, 0.59; % 202, 192, 40;
                0 .45 .74;
                .07 .62 1];

    % Set face colors for each bar
    for i = 1:length(h)
        h(i).FaceColor = f_colors(i,:);
    end

    disp("-------------------------------")
    disp("Average w/o in-floor trajectories")
    disp("Average Length path radius 0.5m: " + num2str(sum(avg_lengths(1,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 1m: " + num2str(sum(avg_lengths(2,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 5m: " + num2str(sum(avg_lengths(3,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 10m: " + num2str(sum(avg_lengths(4,4:end)/(length(avg_lengths)-3))));
    disp("Average Length path radius 30m: " + num2str(sum(avg_lengths(5,4:end)/(length(avg_lengths)-3))));
    disp("-------------------------------")
    disp("Average w in-floor trajectories")
    disp("Average Length path radius 0.5m: " + num2str(sum(avg_lengths(1,:)/(length(avg_lengths)))));
    disp("Average Length path radius 1m: " + num2str(sum(avg_lengths(2,:)/(length(avg_lengths)))));
    disp("Average Length path radius 5m: " + num2str(sum(avg_lengths(3,:)/(length(avg_lengths)))));
    disp("Average Length path radius 10m: " + num2str(sum(avg_lengths(4,:)/(length(avg_lengths)))));
    disp("Average Length path radius 30m: " + num2str(sum(avg_lengths(5,:)/(length(avg_lengths)))));
end

function avg_lengths = calculateAverageLengths(t, floor_combinations)
    avg_lengths = zeros(1, length(floor_combinations));
    counts = zeros(1, length(floor_combinations));
    
    for i = 1:size(t, 1)
        % Parse start and goal points
        startStr = t.StartPoint{i};
        startPoint = str2num(startStr(2:end-1));
        goalStr = t.GoalPoint{i};
        goalPoint = str2num(goalStr(2:end-1));
        
        % Determine start and goal floors
        startFloor = startPoint(3)/100 + 2;
        goalFloor = goalPoint(3)/100 + 2;
        
        % Determine the index for the floor combination
        if startFloor == goalFloor % cases: '2nd-2nd', '3rd-3rd', '4th-4th'
            idx = startFloor - 1;
        else
            if startFloor == 2 
                if goalFloor == 3
                    idx = 4;
                elseif goalFloor == 4
                    idx = 5;
                end
            elseif startFloor == 3
                if goalFloor == 2
                    idx = 6;
                elseif goalFloor == 4
                    idx = 7;
                end
            elseif startFloor == 4
                if goalFloor == 2
                    idx = 8;
                elseif goalFloor == 3
                    idx = 9;
                end
            end
        end
        
        % Add path length to the corresponding combination
        avg_lengths(idx) = avg_lengths(idx) + t.PathLength(i);
        counts(idx) = counts(idx) + 1;
    end
    
    % Calculate averages
    for idx = 1:length(floor_combinations)
        if counts(idx) > 0
            avg_lengths(idx) = avg_lengths(idx) / counts(idx);
        else
            avg_lengths(idx) = NaN;  % or 0, depending on how you want to handle no data
        end
    end
end