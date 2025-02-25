classdef emergencyExitExploration_ES < Navigation
    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
        visionConeHandle % Handle for the vision cone plot
        exitCells % Array to keep track of location of the emergency exits
        signCells % Container of arrays to keep track of location and directions of the emergency signs
        spottedExitCells  % Array to keep track of spotted emergency exits
        spottedSigns  % Array to keep track of spotted signs
        spottedSignsPositions  % Cell array to store positions of spotted signs
        spottedSignsTypes      % Cell array to store types of spotted signs
        goalLineHandle  % Handle for the line between robot and current goal
        originalGoal   % Store the original goal
        visitedSigns  % Array to keep track of visited emergency signs
        observedOccGrid
        isWallFollowing = false
    end 
    
    methods
    
        function bug = emergencyExitExploration_ES(varargin)
            % Bug3_ES Construct a Bug3 navigation object 
            bug = bug@Navigation(varargin{:});
            bug.H = zeros(1, 2);
            bug.j = 1;
            bug.step = 1;
            bug.visionConeHandle = []; % Initialize vision cone handle
            bug.exitCells = []; % Initialize stop cells
            bug.signCells.location = []; % Initialize stop cells
            bug.signCells.direction = []; % Initialize stop cells
            bug.spottedExitCells = []; % Initialize spotted emergency exit cells
            bug.spottedSigns = []; % Initialize spotted emergency sign cells
            bug.spottedSignsPositions = {};
            bug.spottedSignsTypes = {};
            bug.goalLineHandle = [];  % Initialize goal line handle
            bug.originalGoal = [];  % Initialize original goal
            bug.visitedSigns = [];
            bug.observedOccGrid = zeros(size(bug.occgridnav));  % Initialize with unknown space
        end

%% Set Cells Functions 
        function setExitCells(bug, cells) % Set stop cells 
            % cells: Nx2 matrix, where each row is a [x, y] position
            bug.exitCells = cells;
        end

        function setSignCellsLocation(bug, cells)
            % Set stop cells
            % cells: Nx2 matrix, where each row is a [x, y] position
            bug.signCells.location = cells;
        end
    
        function setSignCellsDirection(bug, direction)
            % Set stop cells
            % cells: Nx2 matrix, where each row is a [x, y] position
            bug.signCells.direction = direction;
        end

%% Query Function function
        function [pp, goalReached, exitStairsNumber] = query(bug, start, goal, varargin)
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            goalReached = false;
            exitStairsNumber = 0;
            startHeight = start(3);
            goalHeight = goal(3);
            bug.H = zeros(1, 2);  % Reset H at the beginning of each query
            bug.j = 1;
    
            iter = 0; % count number of next steps done to avoid initial positioning ending with emergency stairs
    
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal = [];
            bug.checkquery(start(1:2), goal(1:2));
    
           if opt.animate
                bug.plot();
                xlim([0,120]);
                axis equal
                hold on;
           end
   
            bug.originalGoal = goal(1:2);  % Store the original goal
    
            % Initial 360-degree rotation to find all points of interest
            bug.perform360Rotation(start(1:2));

            % Set initial goal if any point of interest is found
            bug.updateGoalBasedOnPointsOfInterest(start(1:2), goal(1:2));
    
            % compute the m-line
            bug.mline = homline(bug.start(1), bug.start(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            if opt.animate
%                 bug.plot_mline();
                hold on;
            end
            
            % iterate using the next() method until we reach the goal
            robot = bug.start(:);
            bug.step = 1;
            path = bug.start(:);
    
            while true
                % Check if an emergency exit is visible and go straight towards it
                if bug.checkAndMoveToEmergencyExit(robot)
                    goalReached = true;
                    break;
                end

                iter = iter + 1;
                [closestPoint, pointType] = bug.findClosestPointOfInterest(robot);
                if ~isempty(closestPoint)
                    if isempty(bug.goal) || ~isequal(closestPoint', bug.goal)
                        bug.goal = closestPoint';
                        disp(['Found point of interest: ' pointType ' at (' num2str(closestPoint(1)) ',' num2str(closestPoint(2)) ')']);
                    end
                else
                    % If no point of interest is found, set an arbitrary goal
                    if isempty(closestPoint) && bug.isWallFollowing == false
                        disp("Set Arbitrary Goal Query!")
                        bug.setArbitraryGoal(robot);
                    end
                end
    
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
    
                    if opt.current
                        h = plot(robot(1), robot(2), 'ko', 'MarkerSize', 4);
                    end
                    drawnow
                    if ~isempty(opt.movie)
                        anim.add();
                    end
                    if opt.current
                        delete(h)
                    end
                end
    
                % move to next point on path
                [robot, goalReached, exitStairsNumber] = bug.next(robot, iter, startHeight, goalHeight);
                disp(["Robot Position: " + num2str(robot(1)) + ", " +  num2str(robot(2))]);
                disp(["Goal Reached?: " + num2str(goalReached)]);
                disp(["Exit Stairs Number: " + num2str(exitStairsNumber)]);

                % Append the new point to the path
                if ~isempty(robot)
                    path = [path, robot];
                end
    
                % Check if we've reached the current goal or if we're at an emergency sign
                if isempty(robot) || norm(robot - bug.goal) < 1e-6
                    if isempty(robot)
                        % We've stopped at an emergency sign
                        disp('Stopped at an emergency sign');
                        signIndex = bug.findSignAtPosition(path(:,end));
                        if ~isempty(signIndex)
                            bug.handleEmergencySign(signIndex, path(:,end));
                            robot = bug.start; % Update robot position
                        end
                    else
                        % We've reached the current goal
                        if any(all(abs(bug.exitCells - robot') < 1e-6, 2))
                            % If the goal we've reached is an exit, stop
                            exitStairsNumber = find(all(abs(bug.exitCells - robot') < 1e-6, 2));
                            break;
                        end
                    end
                    
                    % Continue with the original goal or the next point of interest
                    [nextGoal, ~] = bug.findClosestPointOfInterest(robot);
                    if ~isempty(nextGoal)
                        bug.goal = nextGoal';
                    else
                        bug.goal = goal(1:2);
                    end

                    disp(["Next Goal: " + num2str(bug.goal(1)) + ", " +  num2str(bug.goal(2))]);
                    
                    bug.mline = homline(robot(1), robot(2), bug.goal(1), bug.goal(2));
                    bug.mline = bug.mline / norm(bug.mline(1:2));
                    bug.step = 1;
                    bug.j = bug.j + 1;
                    bug.H(bug.j, :) = robot';
                end
            end
            
            if ~isempty(opt.movie)
                anim.close();
            end
    
            % only return the path if required
            if nargout > 0
                pp = path';
            end
        end

%% Plot Functions (Vision Cone, Emergency Sings) 
        function plot_mline(bug, ls)      
            if nargin < 2
                ls = 'k--';
            end
            dims = axis;
            xmin = dims(1); xmax = dims(2);
            ymin = dims(3); ymax = dims(4);
            
            hold on
            if bug.mline(2) == 0
                % handle the case that the line is vertical
                plot([bug.start(1) bug.start(1)], [ymin ymax], 'k--');
            else
                x = [xmin xmax]';
                y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                plot(x, y, ls);
                xlim([0,120]);
                axis equal
            end
        end

        function [handle, coneVertices] = plotVisionCone(bug, robot, heading)
            visionAngle = 170*pi/180;
            visionRadius = 30*4; % 30 meters (4 conversion pxl to meters)
            expansionDistance = 1; % Distance to expand the cone in all directions to draw the cone until the wall
            
            % Increase the number of points for more detail
            numPoints = 100;

            % Plot the vision cone based on the robot's position, angle, and radius
            theta = linspace(-visionAngle/2, visionAngle/2, numPoints)  + heading;

            % Perform ray casting
            [x, y] = bug.rayCast(robot, theta, visionRadius);
            
            % Expand the cone uniformly in all directions
            expandedX = x + expansionDistance * cos(theta);
            expandedY = y + expansionDistance * sin(theta);
            
            % Include the robot position in the cone vertices
            coneVertices = [robot(1), expandedX; robot(2), expandedY];
            
            % Create a closed polygon by adding the robot position at the end
            closedX = [robot(1), expandedX, robot(1)];
            closedY = [robot(2), expandedY, robot(2)];
            
            % Plot the expanded vision cone
            handle = fill(closedX, closedY, 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            
            % Check for visible signs
            bug.checkVisibleSigns(robot, coneVertices);
        end


        function [x, y] = rayCast(bug, start, angles, maxDist)
            x = zeros(1, length(angles));
            y = zeros(1, length(angles));
            
            for i = 1:length(angles)
                [x(i), y(i)] = bug.castSingleRay(start, angles(i), maxDist);
            end
        end

        function [endX, endY] = castSingleRay(bug, start, angle, maxDist)
            dx = cos(angle);
            dy = sin(angle);
            
            for dist = 1:maxDist
                checkX = round(start(1) + dist * dx);
                checkY = round(start(2) + dist * dy);
                
                % Check if the point is within the grid
                if checkX < 1 || checkY < 1 || checkX > size(bug.occgridnav, 2) || checkY > size(bug.occgridnav, 1)
                    break;
                end
                
                % Check if the point is occupied
                if bug.isoccupied([checkX; checkY])
                    break;
                end
            end
            
            endX = start(1) + (dist - 1) * dx;
            endY = start(2) + (dist - 1) * dy;
        end

        function checkVisibleSigns(bug, robot, conePoints)
            % Plot the polygon area in red (for debugging)
            hold on;
            h = fill([robot(1), conePoints(1,:)], [robot(2), conePoints(2,:)], 'r');
            set(h, 'FaceAlpha', 0.2); % Set transparency
            
            % Check for visible exit cells
            for i = 1:size(bug.exitCells, 1)
                if ~ismember(i, bug.spottedExitCells)
                    exitPos = bug.exitCells(i, :);
                    if inpolygon(exitPos(1), exitPos(2), [robot(1), conePoints(1,:)], [robot(2), conePoints(2,:)])                
                        bug.spottedExitCells = [bug.spottedExitCells, i];
                        bug.plotSingleExit(i);
                    end
                end
            end
        
            % Check for visible exit signs
            for i = 1:size(bug.signCells.location, 1)
                if ~ismember(i, bug.spottedSigns)
                    signPos = bug.signCells.location(i, :);
                    if inpolygon(signPos(1), signPos(2), [robot(1), conePoints(1,:)], [robot(2), conePoints(2,:)])
                        bug.spottedSigns = [bug.spottedSigns, i];
                        bug.spottedSignsPositions{end+1} = signPos;
                        bug.spottedSignsTypes{end+1} = bug.signCells.direction{i};
                        bug.plotSingleSign(i);
                    end
                end
            end
            
            % Remove the debug polygon after a short delay
            pause(0.01); % Adjust this value to control how long the polygon is visible
            delete(h);
        end


        function [newGoal, newGoalType] = checkVisiblePointsOfInterest(bug, robot)
            newGoal = [];
            newGoalType = '';
            visiblePoints = [];
            visibleTypes = {};
            coneVertices = bug.visionConeHandle.Vertices;
        
            % Check exit cells
            for i = 1:length(bug.spottedExitCells)
                exitIndex = bug.spottedExitCells(i);
                if inpolygon(bug.exitCells(exitIndex,1), bug.exitCells(exitIndex,2), ...
                             coneVertices(:,1), coneVertices(:,2))
                    visiblePoints = [visiblePoints; bug.exitCells(exitIndex,:)];
                    visibleTypes{end+1} = 'exit';
                end
            end
        
            % Check sign cells
            for i = 1:size(bug.signCells.location, 1)
                if inpolygon(bug.signCells.location(i,1), bug.signCells.location(i,2), ...
                             coneVertices(:,1), coneVertices(:,2))
                    visiblePoints = [visiblePoints; bug.signCells.location(i,:)];
                    visibleTypes{end+1} = 'sign';
                end
            end
        
            % If points of interest are visible, change the goal
            if ~isempty(visiblePoints)
                % First, check if any of the visible points are exits
                exitIndices = strcmp(visibleTypes, 'exit');
                if any(exitIndices)
                    % If there are exits, choose the closest one
                    exitPoints = visiblePoints(exitIndices, :);
                    exitDistances = sqrt(sum((exitPoints - robot').^2, 2));
                    [~, idx] = min(exitDistances);
                    newGoal = exitPoints(idx, :);
                    newGoalType = 'exit';
                else
                    % If no exits, choose the closest sign
                    distances = sqrt(sum((visiblePoints - robot').^2, 2));
                    [~, idx] = min(distances);
                    newGoal = visiblePoints(idx, :);
                    newGoalType = visibleTypes{idx};
                end
            end
        end    
    
        function plotSingleExit(bug, i)
            hold on;
            scatter(bug.exitCells(i, 1), bug.exitCells(i, 2), 15, 'r', 'filled');
        end
    
        function plotSingleSign(bug, i)
            color = bug.getColorByDirection(bug.signCells.direction(i));
            hold on;
            scatter(bug.signCells.location(i, 1), bug.signCells.location(i, 2), 15, 'filled', 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
        end
    
        function color = getColorByDirection(bug, direction)
            switch direction
                case "left"
                    color = 'b';
                case "right"
                    color = "#EDB120";
                case "front"
                    color = 'c';
                case "back"
                    color = 'm';
                otherwise
                    color = 'k';
            end
        end

%% Subroutines Functions
        function perform360Rotation(bug, robot)
            % Perform a 360-degree rotation to scan for points of interest
            for angle = 0:2*pi/180:2*pi  % Rotate in 2-degree increments
                heading = angle;
                if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                    delete(bug.visionConeHandle);
                end
                [bug.visionConeHandle, coneVertices] = bug.plotVisionCone(robot, heading);
                drawnow;
                pause(0.01);
        
                % Update points of interest found during the scan
                bug.updateSpottedPointsOfInterest(robot, coneVertices);
            end
        end

        function updateSpottedPointsOfInterest(bug, robot, coneVertices)
            % Update spotted emergency exits and signs based on vision cone
            bug.checkVisibleSigns(robot, coneVertices);
        end

        function updateGoalBasedOnPointsOfInterest(bug, robot, fallbackGoal)
            % Update the robot's goal based on visible points of interest
            [closestPoint, pointType] = bug.findClosestPointOfInterest(robot);
            if strcmp(pointType, 'exit')
                bug.goal = closestPoint';
            elseif strcmp(pointType, 'sign')
                bug.goal = closestPoint';
            else
                % No point of interest found, set an arbitrary goal
                bug.setArbitraryGoal(robot);
            end
        end

        function setArbitraryGoal(bug, robot)
            % Set an arbitrary goal when no points of interest are found
            disp('No points of interest found. Setting arbitrary goal.');
            
            % Choose "forward" direction (positive y-axis)
            heading = pi/2; % -pi/2, pi, pi/2, 0
            
            % Set a temporary goal 20 units away in the chosen direction
            tempGoalDistance = 100;
            tempGoal = robot + [tempGoalDistance * cos(heading); tempGoalDistance * sin(heading)];
            
            % Ensure the new goal is within the map boundaries
            tempGoal = max(tempGoal, [1; 1]);
            tempGoal = min(tempGoal, [size(bug.occgridnav, 2); size(bug.occgridnav, 1)]);
            
            bug.goal = tempGoal;
            bug.message('New arbitrary goal set: (%d,%d)', round(tempGoal(1)), round(tempGoal(2)));
            
            % Recompute m-line
            bug.mline = homline(robot(1), robot(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            % Reset bug algorithm
            bug.step = 1;
            bug.j = 1;  % Reset hit point counter
            bug.H = robot';  % Reset hit points, starting with current position
            bug.edge = [];  % Clear the edge list
            bug.k = 1;  % Reset edge index
        end

        function [closestPoint, pointType] = findClosestPointOfInterest(bug, robot)
            closestPoint = [];
            pointType = '';
            minDistance = inf;
            currentPosition = robot';
            
            % First, check spotted exit cells (emergency exits)
            for i = 1:length(bug.spottedExitCells)
                exitIndex = bug.spottedExitCells(i);
                distance = norm(bug.exitCells(exitIndex,:) - currentPosition);
                if distance < minDistance && distance > 1e-6  % Ensure it's not the current position
                    minDistance = distance;
                    closestPoint = bug.exitCells(exitIndex,:);
                    pointType = 'exit';
                end
            end
        
            % If an emergency exit was found, return it immediately
            if ~isempty(closestPoint)
                return;
            end
        
            % If no emergency exit was found, then check spotted signs
            for i = 1:length(bug.spottedSigns)
                signIndex = bug.spottedSigns(i);
                % Check if this sign has not been visited
                if ~ismember(signIndex, bug.visitedSigns)
                    signPos = bug.signCells.location(signIndex, :);
                    distance = norm(signPos - currentPosition);
                    if distance < minDistance && distance > 1e-6  % Ensure it's not the current position
                        minDistance = distance;
                        closestPoint = signPos;
                        pointType = bug.signCells.direction{signIndex};
                    end
                end
            end
        
            if isempty(closestPoint)
                disp('No unvisited point of interest found');
            else
                disp(['Found point of interest: ' pointType ' at (' num2str(closestPoint(1)) ',' num2str(closestPoint(2)) ')']);
            end
        end

        function found = checkAndMoveToEmergencyExit(bug, robot)
            % Check if an emergency exit is visible and move towards it
            found = false;
            for i = 1:length(bug.spottedExitCells)
                exitPos = bug.exitCells(bug.spottedExitCells(i), :);
                if norm(robot' - exitPos) < 1e-6  % Close enough to move directly
                    bug.goal = exitPos';
                    found = true;
                    return;
                end
            end
        end
        
        function handleEmergencySign(bug, signIndex, robot)
            % Process actions upon reaching an emergency sign
            bug.visitedSigns = [bug.visitedSigns, signIndex]; % Mark sign as visited
            direction = bug.signCells.direction{signIndex}; % Get direction suggested by the sign
            disp("Following direction: " + direction);
            
            % Get the angle for the suggested direction
            heading = bug.getDirectionAngle(direction);
            
            % Move the robot in the suggested direction
            moveDistance = 1; % Move 1 unit in the suggested direction
            newRobotPos = robot + [moveDistance * cos(heading); moveDistance * sin(heading)];
            
            % Ensure the new position is within the map boundaries
            newRobotPos = max(newRobotPos, [1; 1]);
            newRobotPos = min(newRobotPos, [size(bug.occgridnav, 2); size(bug.occgridnav, 1)]);
            
            % Update the robot's position
            bug.start = newRobotPos;
            
            % Rotate and look for new points of interest in the sign's suggested direction
            bug.rotateTowardsDirection(newRobotPos, heading);
            
            % Check for visible emergency exits first
            visibleExits = bug.checkVisibleEmergencyExits(newRobotPos);
            if ~isempty(visibleExits)
                % If there are visible exits, choose the closest one
                exitDistances = sqrt(sum((visibleExits - newRobotPos').^2, 2));
                [~, idx] = min(exitDistances);
                closestExit = visibleExits(idx, :);
                bug.goal = closestExit';
                disp("Emergency exit found! Updated goal to exit at (" + num2str(closestExit(1)) + "," + num2str(closestExit(2)) + ")");
            else
                % If no emergency exits are visible, find the next closest point of interest
                [nextGoal, pointType] = bug.findClosestPointOfInterest(newRobotPos);
                if ~isempty(nextGoal)
                    bug.goal = nextGoal'; % Update goal to the next point of interest
                    disp("Updated goal to: " + pointType + " at (" + num2str(nextGoal(1)) + "," + num2str(nextGoal(2)) + ")");
                else
                    % If no point of interest is visible, set a temporary goal in the suggested direction
                    tempGoalDistance = 50; % Set a temporary goal 50 units away in the suggested direction
                    bug.goal = newRobotPos + [tempGoalDistance * cos(heading); tempGoalDistance * sin(heading)];
                    disp("No points of interest visible, setting temporary goal in direction: " + num2str(heading));
                end
            end
            
            % Recompute m-line
            bug.mline = homline(newRobotPos(1), newRobotPos(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            % Reset bug algorithm
            bug.step = 1;
            bug.j = 1;  % Reset hit point counter
            bug.H = newRobotPos';  % Reset hit points, starting with new position
            bug.edge = [];  % Clear the edge list
            bug.k = 1;  % Reset edge index
        end

        function visibleExits = checkVisibleEmergencyExits(bug, robot)
            visibleExits = [];
            if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                coneVertices = bug.visionConeHandle.Vertices;
                for i = 1:length(bug.spottedExitCells)
                    exitIndex = bug.spottedExitCells(i);
                    if inpolygon(bug.exitCells(exitIndex,1), bug.exitCells(exitIndex,2), ...
                                 coneVertices(:,1), coneVertices(:,2))
                        visibleExits = [visibleExits; bug.exitCells(exitIndex,:)];
                    end
                end
            end
        end
        
        function angle = getDirectionAngle(~, direction)
            % Map direction strings to angles (assumes robot's initial heading is along +x axis)
            switch lower(direction)
                case "left"
                    angle = pi;
                case "right"
                    angle = 0;
                case "front"
                    angle = pi/2;
                case "back"
                    angle = -pi/2;
                otherwise
                    error("Unknown direction: " + direction);
            end
        end
        
        function rotateTowardsDirection(bug, robot, heading)
            % Rotate and look for points of interest in the given direction
            if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                delete(bug.visionConeHandle);
            end
            [bug.visionConeHandle, coneVertices] = bug.plotVisionCone(robot, heading);
            drawnow;
            
            % Update points of interest based on the vision cone
            bug.updateSpottedPointsOfInterest(robot, coneVertices);
        end
        
        function signIndex = findSignAtPosition(bug, position)
            % Check if the current position matches any emergency sign location
            signIndex = find(all(abs(bug.signCells.location - position') < 1e-6, 2), 1);
        end

%% Next Function function
    function [n, goalReached, exitStairsNumber] = next(bug, robot, iter, startHeight, goalHeight)
            %% NEXT.STEP.INIT
            % Implement the main state machine for Bug3
            n = [];
            goalReached = false;
            exitStairsNumber = 0;
            robot = robot(:);   % These are coordinates (x, y)


            % Check if we're moving towards an emergency exit
            movingTowardsExit = ~isempty(bug.spottedExitCells) && isequal(bug.goal, bug.exitCells(bug.spottedExitCells(1), :)');
        

            % Check if vision cone handle exists and is valid
            if isempty(bug.visionConeHandle) || ~isvalid(bug.visionConeHandle)
                % If not, create an initial vision cone
                heading = atan2(bug.goal(2) - robot(2), bug.goal(1) - robot(1));
                bug.visionConeHandle = bug.plotVisionCone(robot, heading);
            end
        
            % Check if current position matches any stop cells (emergency exits)
            if ismember(robot', bug.exitCells, 'rows') && iter > 5
                exitStairsNumber = find(all(abs(bug.exitCells - robot') < 1e-6, 2));
                bug.message('Stopped at predefined exit cell: (%d,%d)', robot);
                n = []; % Stop the algorithm
                return;
            end

            % Check if the current position matches any sign cell
            signIndex = find(all(abs(bug.signCells.location - robot') < 1e-6, 2), 1);
            if ~isempty(signIndex)
                disp("Stopped at predefined emergency sign cell: X="+ num2str(robot(1)) + ", Y=" + num2str(robot(2)));
                bug.handleEmergencySign(signIndex, robot);
                n = bug.start; % Set next position to the updated start position
                return;
            end

            % Check for visible points of interest and update goal if necessary
            [newGoal, newGoalType] = bug.checkVisiblePointsOfInterest(robot);
            if ~isempty(newGoal)
                if strcmp(newGoalType, 'exit')
                    bug.goal = newGoal';
                    bug.message('New goal set: Emergency exit at (%d,%d)', newGoal(1), newGoal(2));
                elseif isempty(bug.spottedExitCells)  % Only change goal to a sign if no exit has been spotted
                    bug.goal = newGoal';
                    bug.message('New goal set: Emergency sign at (%d,%d)', newGoal(1), newGoal(2));
                end
                
                % Recompute m-line
                bug.mline = homline(robot(1), robot(2), bug.goal(1), bug.goal(2));
                bug.mline = bug.mline / norm(bug.mline(1:2));
                
                % Reset bug algorithm
                bug.step = 1;
                bug.j = 1;  % Reset hit point counter
                bug.H = robot';  % Reset hit points, starting with current position
                bug.edge = [];  % Clear the edge list
                bug.k = 1;  % Reset edge index
            end

            % Update the goal line
            if isempty(bug.goalLineHandle) || ~isvalid(bug.goalLineHandle)
                bug.goalLineHandle = line([robot(1), bug.goal(1)], [robot(2), bug.goal(2)], 'Color', 'r', 'LineStyle', '--');
            else
                set(bug.goalLineHandle, 'XData', [robot(1), bug.goal(1)], 'YData', [robot(2), bug.goal(2)]);
            end

            %% NEXT.STEP.1:
            if bug.step == 1
                disp('Step 1: Moving towards goal');
                % Step 1. Move along the M-line toward the goal
                if colnorm(bug.goal - robot) == 0
                    goalReached = true;
                    disp('Goal reached on the current floor.');
                    return
                end

                % Motion on the line toward goal
                d = bug.goal - robot;    
    
                if abs(d(1)) > abs(d(2))
                    % Line slope less than 45 degrees
                    dx = sign(d(1));
                    L = bug.mline;
                    if L(2) ~= 0
                        y = -((robot(1) + dx) * L(1) + L(3)) / L(2);
                        dy = round(y - robot(2));
                    else
                        % Vertical line case
                        dy = sign(d(2));
                    end
                else
                    % Line slope greater than 45 degrees
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -((robot(2) + dy) * L(2) + L(3)) / L(1);
                    dx = round(x - robot(1));
                end
        
                % Detect if next step is an obstacle
                if bug.isoccupied(robot + [dx; dy])
                    disp('Obstacle detected! Transitioning to Step 2 (obstacle avoidance)');
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j, :) = robot; % Define hit point
                    bug.step = 2;
                    % Get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgridnav == 0, robot);
                    bug.k = 2;  % Skip the first edge point, we are already there

                    bug.isWallFollowing = true;
                    n = robot; % Stay in place for this step
                else
                    n = robot + [dx; dy];
    
                    % Plot the vision cone after moving
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));
    
                    % Delete previous vision cone if it exists
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end
    
                    % Plot the new vision cone
                    bug.visionConeHandle = bug.plotVisionCone(n, heading);
                end   
            end % Step 1

            %% NEXT.STEP.2:
            if bug.step == 2
                disp('Step 2: Obstacle avoidance');

                bug.isWallFollowing = true;

                % Step 2. Move around the obstacle until we reach a point on the M-line closer than when we started.
                if colnorm(bug.goal - robot) == 0 % Are we there yet?
                    return
                end

                if bug.k <= numcols(bug.edge)
                    n = bug.edge(:, bug.k);  % Next edge point

                    if ~movingTowardsExit  % Check for visible points of interest while wall-following
                        [newGoal, newGoalType] = bug.checkVisiblePointsOfInterest(n);
                        if ~isempty(newGoal)
                            % If a point of interest is found, update the goal
                            bug.updateGoalBasedOnPointsOfInterest(n, bug.originalGoal);
                            bug.step = 1;  % Go back to step 1 to move towards the new goal
                            return;
                        end
                    end

                    % Plot the vision cone after moving
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));
    
                    % Delete previous vision cone if it exists
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end
    
                    % Plot the new vision cone
                    bug.visionConeHandle = bug.plotVisionCone(n, heading);

                else
                    % We are at the end of the list of edge points, we are back where we started. Step 2.c test.
                    disp('Robot is trapped');
                    error('RTB:bug2:noplan', 'Robot is trapped')    
                    return;
                end
    
                % Are we on the M-line now?
                if abs([robot' 1] * bug.mline') <= 0.5
                    bug.message('(%d,%d) moving along the M-line', n);
                    % Are we closer than when we encountered the obstacle?
                    if colnorm(robot - bug.goal) < colnorm(bug.H(bug.j, :)' - bug.goal)
                        disp('Back to moving along the M-line (Step 1)');
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        return;
                    end
                end
                % No, keep going around
                bug.message('(%d,%d) keep moving around obstacle', n)
                bug.k = bug.k + 1;
            end % Step 2

        end % next

        function plan(bug)
            error('RTB:Bug2:badcall', 'This class has no plan method');
        end
    end
end