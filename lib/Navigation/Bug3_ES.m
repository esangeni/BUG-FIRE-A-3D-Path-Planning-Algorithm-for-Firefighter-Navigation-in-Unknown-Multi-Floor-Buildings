classdef Bug3_ES < Navigation

    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
        visionConeHandle % Handle for the vision cone plot
        exitCells
        signCells
    end

    methods

        function bug = Bug3_ES(varargin)
            % Bug3_ES Construct a Bug3 navigation object 
            bug = bug@Navigation(varargin{:});
            bug.H = [];
            bug.j = 1;
            bug.step = 1;
            bug.visionConeHandle = []; % Initialize vision cone handle
            bug.exitCells = []; % Initialize stop cells
            bug.signCells.location = []; % Initialize stop cells
        end

        function setExitCells(bug, cells)
            % Set stop cells
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

        function [pp, goalReached, exitStairsNumber] = query(bug, start, goal, varargin)
         
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            goalReached = false;
            exitStairsNumber = 0;
            startHeight = start(3);
            goalHeight = goal(3);

            iter = 0; % count number of next steps done to avoid initial positioning ending with emergency stairs

            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal = [];
            bug.checkquery(start(1:2), goal(1:2));
            
            % compute the m-line
            %  create homogeneous representation of the line line*[x y 1]' = 0
            bug.mline = homline(bug.start(1), bug.start(2), bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            if opt.animate
                bug.plot();
                bug.plot_mline();
                bug.plotExitCells();  % Plot the stop cells in blue
            end
            
            % iterate using the next() method until we reach the goal
            robot = bug.start(:);
            bug.step = 1;
            path = bug.start(:);

            while true
                iter = iter + 1;

                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);

                    if opt.current
                        h = plot(robot(1), robot(2), 'ko', 'MarkerSize', 8);
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

                % are we there yet?
                if isempty(robot)
                    % yes, exit the loop
                    break
                else
                    % no, append it to the path
                    path = [path robot(:)];
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
                    plot([start(1) start(1)], [ymin ymax], 'k--');
                else
                    x = [xmin xmax]';
                    y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                    plot(x, y, ls);
                    xlim([0,120]);
                    axis equal
                end
        end

        function handle = plotVisionCone(bug, robot, heading)
            visionAngle = 45*pi/180;
            visionRadius = 30*4; % 30 meters (4 conversion pxl to meters)
            % Plot the vision cone based on the robot's position, angle, and radius
            theta = linspace(-visionAngle/2, visionAngle/2, 30)  + heading;
            x = robot(1) + visionRadius * cos(theta);
            y = robot(2) + visionRadius * sin(theta);
            handle = fill([robot(1), x], [robot(2), y], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % Return handle
        end

        function plotExitCells(bug)
            % Plot the stop cells in blue
            if ~isempty(bug.exitCells)
                hold on;
                scatter(bug.exitCells(:, 1), bug.exitCells(:, 2), 15, 'blue', 'filled');
            end
        end
        
        function [n, goalReached, exitStairsNumber] = next(bug, robot, iter, startHeight, goalHeight)

            % Implement the main state machine for Bug3
            n = [];
            goalReached = false;
            exitStairsNumber = 0;
            robot = robot(:);   % These are coordinates (x, y)

            % Check if current position matches any stop cells
            if ismember(robot', bug.exitCells, 'rows') && iter > 5
                if(robot' == bug.exitCells(1,:))
                    exitStairsNumber = 1;
                elseif(robot' == bug.exitCells(2,:))
                    exitStairsNumber = 2;
                elseif(robot' == bug.exitCells(3,:))
                    exitStairsNumber = 3;
                else
                    disp('None of the exit stairs are defined.')
                end
                bug.message('Stopped at predefined exit cell: (%d,%d)', robot);
                n = []; % Stop the algorithm
                return;
            end

            if bug.step == 1
                % Step 1. Move along the M-line toward the goal
                if colnorm(bug.goal - robot) == 0 && startHeight == goalHeight % Are we there yet?
                    goalReached = true;
                    disp('Goal reached.');
                    return
                end

                if colnorm(bug.goal - robot) == 0 && startHeight ~= goalHeight % Are we there yet?
                    goalReached = true;
                    disp('Goal projected reached.');
                    return
                end
        
                % Motion on the line toward goal
                d = bug.goal - robot;
                if abs(d(1)) > abs(d(2))
                    % Line slope less than 45 degrees
                    dx = sign(d(1));
                    L = bug.mline;
                    y = -((robot(1) + dx) * L(1) + L(3)) / L(2);
                    dy = round(y - robot(2));
                else
                    % Line slope greater than 45 degrees
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -((robot(2) + dy) * L(2) + L(3)) / L(1);
                    dx = round(x - robot(1));
                end
        
                % Detect if next step is an obstacle
                if bug.isoccupied(robot + [dx; dy])
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j, :) = robot; % Define hit point
                    bug.step = 2;
                    % Get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgridnav == 0, robot);
                    bug.k = 2;  % Skip the first edge point, we are already there
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
        
            if bug.step == 2
                % Step 2. Move around the obstacle until we reach a point on the M-line closer than when we started.
                if colnorm(bug.goal - robot) == 0 % Are we there yet?
                    return
                end
        
                if bug.k <= numcols(bug.edge)
                    n = bug.edge(:, bug.k);  % Next edge point

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
                        % Back to moving along the M-line
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

    end % methods
end % classdef

% NFPA guidelines:
% The National Fire Protection Association (NFPA) Life Safety Code 
% specifies this 100-feet (30 meters) viewing distance for exit signs.
% Meaning no point in an exit access corridor should be further than 100 
% feet from the nearest visible exit sign; this distance can vary slightly 
% depending on the sign's size and illumination capabilities.

