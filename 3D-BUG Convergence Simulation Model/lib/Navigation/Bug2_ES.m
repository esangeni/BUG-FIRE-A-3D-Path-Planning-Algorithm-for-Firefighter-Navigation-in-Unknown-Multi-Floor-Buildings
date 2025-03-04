%BUG2 Bug navigation class
%
% A concrete subclass of the abstract Navigation class that implements the bug2 
% navigation algorithm.  This is a simple automaton that performs local 
% planning, that is, it can only sense the immediate presence of an obstacle.
%
% Methods::
%   Bug2        Constructor
%   query       Find a path from start to goal
%   plot        Display the obstacle map
%   display     Display state/parameters in human readable form
%   char        Convert to string
%
% Example::
%         load map1             % load the map
%         bug = Bug2(map);      % create navigation object
%         start = [20,10]; 
%         goal = [50,35];
%         bug.query(start, goal);   % animate path
%
% Reference::
% -  Dynamic path planning for a mobile automaton with limited information on the environment,,
%    V. Lumelsky and A. Stepanov, 
%    IEEE Transactions on Automatic Control, vol. 31, pp. 1058-1063, Nov. 1986.
% -  Robotics, Vision & Control, Sec 5.1.2,
%    Peter Corke, Springer, 2011.
%  
% See also Navigation, DXform, Dstar, PRM.



% Copyright (C) 1993-2017, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

classdef Bug2_ES < Navigation

    properties(Access=protected)
        H       % hit points
        j       % number of hit points
        mline   % line from starting position to goal
        step    % state, in step 1 or step 2 of algorithm
        edge    % edge list
        k       % edge index
        visionConeHandle % Handle for the vision cone plot
    end

    methods

        function bug = Bug2_ES(varargin)
            %Bug2.Bug2 Construct a Bug2 navigation object 
            %
            % B = Bug2(MAP, OPTIONS) is a bug2 navigation object, and MAP is an occupancy grid,
            % a representation of a planar world as a matrix whose elements are 0 (free
            % space) or 1 (occupied).
            %
            % Options::
            % 'goal',G      Specify the goal point (1x2)
            % 'inflate',K   Inflate all obstacles by K cells.
            %
            % See also Navigation.Navigation.

            % invoke the superclass constructor
            bug = bug@Navigation(varargin{:});
            bug.H = [];
            bug.j = 1;
            bug.step = 1;
            bug.visionConeHandle = []; % Initialize vision cone handle
        end

        function [pp, goalReached] = query(bug, start, goal, varargin)
            %Bug2.query  Find a path
            %
            % B.query(START, GOAL, OPTIONS) is the path (Nx2) from START (1x2) to GOAL
            % (1x2).  Row are the coordinates of successive points along the path.  If
            % either START or GOAL is [] the grid map is displayed and the user is
            % prompted to select a point by clicking on the plot.
            %
            % Options::
            %  'animate'   show a simulation of the robot moving along the path
            %  'movie',M   create a movie
            %  'current'   show the current position position as a black circle
            %
            % Notes::
            % - START and GOAL are given as X,Y coordinates in the grid map, not as
            %   MATLAB row and column coordinates.
            % - START and GOAL are tested to ensure they lie in free space.
            % - The Bug2 algorithm is completely reactive so there is no planning
            %   method.
            % - If the bug does a lot of back tracking it's hard to see the current
            %   position, use the 'current' option.
            % - For the movie option if M contains an extension a movie file with that
            %   extension is created.  Otherwise a folder will be created containing
            %   individual frames.
            %
            % See also Animate.
         
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            goalReached = false;
            
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal = [];
            bug.checkquery(start, goal);
            
            % compute the m-line
            %  create homogeneous representation of the line
            %  line*[x y 1]' = 0
            bug.mline = homline(bug.start(1), bug.start(2), ...
                bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            if opt.animate
                bug.plot();
                bug.plot_mline();
            end
            
            % iterate using the next() method until we reach the goal
            robot = bug.start(:);
            bug.step = 1;
            path = bug.start(:);
            while true
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
                [robot, goalReached] = bug.next(robot);

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
            
                % parameters of the M-line, direct from initial position to goal
                % as a vector mline, such that [robot 1]*mline = 0
                
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
                    axis equal
                end
        end

        function handle = plotVisionCone(bug, robot, heading)
            visionAngle = 45*pi/180;
            visionRadius = 30*4; % 30 meters (4 conversion pxl to meters)
            % Plot the vision cone based on the robot's position, angle, and radius
            theta = linspace(-visionAngle/2, visionAngle/2, 30)  + heading;
%             x = robot(1) + visionRadius * cos(theta);
%             y = robot(2) + visionRadius * sin(theta);
            [x, y] = bug.rayCast(robot, theta, visionRadius);
            handle = fill([robot(1), x], [robot(2), y], 'k', 'FaceAlpha', 0.2, 'EdgeColor', 'none'); % Return handle
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

        function [n, goalReached] = next(bug, robot)
            
            % implement the main state machine for bug2
            n = [];
            goalReached = false;
            robot = robot(:);
            % these are coordinates (x,y)
          
            if bug.step == 1
                % Step 1.  Move along the M-line toward the goal

                if colnorm(bug.goal - robot) == 0 % are we there yet?
                    goalReached = true;
                    disp('Goal reached.');
                    return
                end

%                 % motion on line toward goal
%                 d = bug.goal-robot;
%                 if abs(d(1)) > abs(d(2))
%                     % line slope less than 45 deg
%                     dx = sign(d(1));
%                     L = bug.mline;
%                     y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
%                     dy = round(y - robot(2));
%                 else
%                     % line slope greater than 45 deg
%                     dy = sign(d(2));
%                     L = bug.mline;
%                     x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
%                     dx = round(x - robot(1));
%                 end

                % Motion on the line toward goal
                d = bug.goal - robot;    
    
                if abs(d(1)) > abs(d(2))
                    % Line slope less than 45 degrees
                    dx = sign(d(1));
                    L = bug.mline;
                    if L(2) >= 1e-5 || L(2) <= -1e-5
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

                % detect if next step is an obstacle
                if bug.isoccupied(robot + [dx; dy])
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j,:) = robot; % define hit point
                    bug.step = 2;
                    % get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgridnav == 0, robot);
                    bug.k = 2;  % skip the first edge point, we are already there
                else
                    n = robot + [dx; dy];

                    % Plot the vision cone after moving
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));

                    % Delete previous vision cone if it exists
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end

%                     % Plot the new vision cone
%                     bug.visionConeHandle = bug.plotVisionCone(n, heading);
                end
            end % step 1

            if bug.step == 2
                % Step 2.  Move around the obstacle until we reach a point
                % on the M-line closer than when we started.
                if colnorm(bug.goal-robot) == 0 % are we there yet?
                    return
                end

                if bug.k <= numcols(bug.edge)
                    n = bug.edge(:,bug.k);  % next edge point

                    % Plot the vision cone after moving
                    delta = n - robot;
                    heading = atan2(delta(2), delta(1));

                    % Delete previous vision cone if it exists
                    if ~isempty(bug.visionConeHandle) && isvalid(bug.visionConeHandle)
                        delete(bug.visionConeHandle);
                    end

%                     % Plot the new vision cone
%                     bug.visionConeHandle = bug.plotVisionCone(n, heading);
                else
                    % we are at the end of the list of edge points, we
                    % are back where we started.  Step 2.c test.
%                     error('RTB:bug2:noplan', 'robot is trapped')
                    return;
                end

                % are we on the M-line now ?
                if abs( [robot' 1]*bug.mline') <= 0.9 % 0.7 EUDALD CHANGED THRESHOLD FOR COUNTING M-LINE CROSSING SQUARED OBJECTS VERTEXS: OLD "0.5"
                    bug.message('(%d,%d) moving along the M-line', n);
                    % are closer than when we encountered the obstacle?
                    if colnorm(robot-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
                        % back to moving along the M-line
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        return;
                    end
                end
                % no, keep going around
                bug.message('(%d,%d) keep moving around obstacle', n)
                bug.k = bug.k+1;
            end % step 2
        end % next
        
        function plan(bug)
            error('RTB:Bug2:badcall', 'This class has no plan method');
        end

    end % methods
end % classdef
