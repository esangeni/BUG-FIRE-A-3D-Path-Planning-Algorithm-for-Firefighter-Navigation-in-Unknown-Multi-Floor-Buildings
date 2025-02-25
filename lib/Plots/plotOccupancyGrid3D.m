function plotOccupancyGrid3D(occupancyGrid, zLevel)
% Function to plot the occupancy grid
    [rows, cols] = size(occupancyGrid);
    hold on;
    for r = 1:rows
        for c = 1:cols
            if occupancyGrid(r, c) == 1
                x = [c - 0.5, c + 0.5]; % Line extends horizontally
                y = [r - 0.5, r - 0.5]; % Line extends vertically
                plot3(x, y, zLevel, 'k-', 'LineWidth', 2); % Black walls
            end
        end
    end
end