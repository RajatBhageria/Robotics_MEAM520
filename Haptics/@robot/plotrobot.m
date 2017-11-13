%PLOT Graphical robot animation
%
%	PLOT(ROBOT, Q)
%	PLOT(ROBOT, Q, options)
%
% Displays a graphical animation of a robot based on the
% kinematic model.  A stick figure polyline joins the origins of
% the link coordinate frames.
% The robot is displayed at the joint angle Q, or if a matrix it is
% animated as the robot moves along the trajectory.
%
% The graphical robot object holds a copy of the robot object and
% the graphical element is tagged with the robot's name (.name method).
% This state also holds the last joint configuration which can be retrieved,
% see PLOT(robot) below.
%
% If no robot of this name is currently displayed then a robot will
% be drawn in the current figure.  If hold is enabled (hold on) then the
% robot will be added to the current figure.
%
% If the robot already exists then that graphical model will be found 
% and moved.
%
% MULTIPLE VIEWS OF THE SAME ROBOT
% If one or more plots of this robot already exist then these will all
% be moved according to the argument Q.  All robots in all windows with 
% the same name will be moved.
%
% MULTIPLE ROBOTS
% Multiple robots can be displayed in the same plot, by using "hold on"
% before calls to plot(robot).  
%
% options is a list of any of the following:
% 'workspace' [xmin, xmax ymin ymax zmin zmax]
% 'perspective' 'ortho'		controls camera view mode
% 'erase' 'noerase'		controls erasure of arm during animation
% 'loop' 'noloop'		controls endless loop mode
% 'base' 'nobase'		controls display of base 'pedestal'
% 'wrist' 'nowrist'		controls display of wrist
% 'name', 'noname'		display the robot's name 
% 'shadow' 'noshadow'		controls display of shadow
% 'xyz' 'noa'			wrist axis label
% 'joints' 'nojoints'		controls display of joints
% 'mag' scale			annotation scale factor
%
% The options come from 3 sources and are processed in the order:
% 1. Cell array of options returned by the function PLOTBOTOPT
% 2. Cell array of options returned by the .plotopt method of the
%    robot object.  These are set by the .plotopt method.
% 3. List of arguments in the command line.
%
% GRAPHICAL ANNOTATIONS:
%
% The basic stick figure robot can be annotated with
%  shadow on the floor
%  XYZ wrist axes and labels
%  joint cylinders and axes
%
% All of these require some kind of dimension and this is determined
% using a simple heuristic from the workspace dimensions.  This dimension
% can be changed by setting the multiplicative scale factor using the
% 'mag' option above.
%
% GETTING GRAPHICAL ROBOT STATE:
% q = PLOT(ROBOT)
% Returns the joint configuration from the state of an existing 
% graphical representation of the specified robot.  If multiple
% instances exist, that of the first one returned by findobj() is
% given.
%
% MOVING JUST ONE INSTANCE oF A ROBOT:
%
%  r = PLOT(robot, q)
%
% Returns a copy of the robot object, with the .handle element set.
%
%  PLOT(r, q)
%
% will animate just this instance, not all robots of the same name.
%
% See also: PLOTBOTOPT, DRIVEBOT, FKINE, ROBOT.


% HANDLES:
%
%  A robot comprises a bunch of individual graphical elements and these are 
% kept in a structure which can be stored within the .handle element of a
% robot object:
%	h.robot		the robot stick figure
%	h.shadow	the robot's shadow
%	h.x		wrist vectors
%	h.y
%	h.z
%	h.xt		wrist vector labels
%	h.yt
%	h.zt
%
%  The plot function returns a new robot object with the handle element set.
%
% For the h.robot object we additionally: 
%	- save this new robot object as its UserData
%	- tag it with the name field from the robot object
%
%  This enables us to find all robots with a given name, in all figures,
% and update them.

% Copyright (C) 1993-2008, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for Matlab (RTB).
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

function rnew = plotrobot(robot, tg, skipFrame)

	%
	% q = PLOT(robot)
	% return joint coordinates from a graphical robot of given name
	%
    if nargin < 2
		rh = findobj('Tag', robot.name);
		if ~isempty(rh)
			r = get(rh(1), 'UserData');
			rnew = r.q;
            % Input changed to inches, no need to convert
%             rnew(6) = rnew(6)*25.4; % Convert inches to mm (r.q returns in, but joing variable defined in mm)
            rnew = rnew + [0 pi/2 -pi/2 pi/2 0 0]; % r.q returns total theta, but many joints have an offset, adjust for offset
		end
		return
    end
    
    if nargin < 3
        skipFrame = false;
    end
    
    % process options
	opt = plot_options(robot);

	%
	% robot2 = ROBOT(robot, q, varargin)
	%
    % Input chaged to in, so no need to convert
%     tg(6) = tg(6)/25.4; % Convert mm to in (animate expects in, but joint variable defined in mm)
    tg = tg + [0 -pi/2 pi/2 -pi/2 0 0]; % animate expects total theta, but many joints have an offset, adjust for offset
    np = size(tg,1);
	%np = numrows(tg);
	n = robot.n;

	%if numcols(tg) ~= n,
    if size(tg,2) ~= n
		error('Insufficient columns in q')
    end

    if ~isempty(robot.handle)
        %disp('has handles')
		% handles provided, animate just that robot
		for r=1:opt.repeat
		    for p=1:np
                animate( robot, tg(p,:));
                pause(opt.delay)
		    end
		end

		return;
    end

	% Do the right thing with figure windows.
    ax = gca;
    
    % if this figure has no robot in it, create one
    if isempty( findobj(ax, 'Tag', robot.name) )

		h = create_new_robot(robot, opt);

		% save the handles in the passed robot object, and
		% attach it to the robot as user data.
		robot.handle = h;
		set(h.robot, 'Tag', robot.name);
		set(h.robot, 'UserData', robot);
    end

    % get handle of any existing robot of same name
	rh = findobj('Tag', robot.name);
    
	% now animate all robots tagged with this name
	for rep=1:opt.repeat
	    for p=1:np
            for r = rh'
                if ~skipFrame
                    animate( get(r, 'UserData'), tg(p,:));
                end
            end
	    end
	end

	% save the last joint angles away in the graphical robot
	for r=rh',
		rr = get(r, 'UserData');
		rr.q = tg(end,:);
		set(r, 'UserData', rr);
	end

	if nargout > 0,
		rnew = robot;
	end

%PLOT_OPTIONS
%
%	o = PLOT_OPTIONS(robot, options)
%
% Returns an options structure

function o = plot_options(robot)
	%%%%%%%%%%%%%% process options
  	o.erasemode = 'normal';
	o.joints = robot.joints;
	o.repeat = 1;
	o.shadow = robot.shadow;
	o.dims = [];
	o.magscale = 1;
	o.name = 1;
    o.jointaxis = robot.jointaxis;
    if isempty(o.dims),
		%
		% simple heuristic to figure the maximum reach of the robot
		%
		L = robot.link;
		reach = 0;
		for i=1:robot.n,
			reach = reach + abs(L{i}.A) + abs(L{i}.D);
		end
		o.dims = [-reach reach -reach reach 0 reach*5/3]*0.6;
		o.mag = reach/10;
    else
		reach = min(abs(dims));
    end
	o.mag = o.magscale * reach/10;

%CREATE_NEW_ROBOT
% 
%	h = CREATE_NEW_ROBOT(robot, opt)
%
% Using data from robot object and options create a graphical robot in
% the current figure.
%
% Returns a structure of handles to graphical objects.
%
% If current figure is empty, draw robot in it
% If current figure has hold on, add robot to it
% Otherwise, create new figure and draw robot in it.
%	

function h = create_new_robot(robot, opt)
    if ~isempty(robot.toolhandle)
        h.tool = robot.toolhandle;
        set(h.tool, 'erasemode', opt.erasemode);
    end

	h.mag = opt.mag;

	%
	% setup an axis in which to animate the robot
	%
    figure(1)
    axis(opt.dims);
    
    xlabel('X (in.)')
    ylabel('Y (in.)')
    zlabel('Z (in.)')
    set(gca,'xtick',30*[-3/6 -2/6 -1/6 0 1/6 2/6 3/6], 'ytick',30*[-3/6 -2/6 -1/6 0 1/6 2/6 3/6],'ztick',20*[-.5 -.25 0 .25 .5 .75 1])
    grid on
    view(80,20)
    
    axis equal vis3d
    xlim(30*0.5*[-2/3 1]);
    ylim(30*0.5*[-1 1]);
    zlim([-10 20]);
    
	zlims = get(gca, 'ZLim');
	h.zmin = zlims(1);

	% create a line which we will
	% subsequently modify.
	%
	h.robot = line(robot.lineopt{:}, ...
		 'Color', [0 0 0]);
    if opt.shadow == 1,
		h.shadow = line(robot.shadowopt{:}, ...
            'Color', 0.6*ones(1,3));
    end

	%
	% display cylinders (revolute) or boxes (pristmatic) at
	% each joint, as well as axis centerline.
	%
	
    L = robot.link;
    for i=(1:robot.n-1),
        if opt.joints == 1,
			% cylinder or box to represent the joint
            if L{i}.sigma == 0,
				N = 20;
            elseif L{i}.sigma == 1
				N = 4;
            end
            
            wrist = robot.sphericalwrist;
            if any(i == wrist) && i == wrist(2)
                [xc, yc, zc] = sphere(N);
                xc = opt.mag/3 * xc;
                yc = opt.mag/3 * yc;
                zc = opt.mag/3 * zc;
            else
                [xc,yc,zc] = cylinder(opt.mag/4, N);
                zc(zc==0) = -opt.mag/3;
                zc(zc==1) = opt.mag/3;
            end
            
            actuatoroffset = L{i}.actuatoroffset;
            xc = xc + actuatoroffset(1);
            yc = yc + actuatoroffset(2);
            zc = zc + actuatoroffset(3);

			% add the surface object and color it
            if any(i == wrist)
                if i == wrist(2)
                    h.joint(i) = surface(xc,yc,zc);
                end
            else
                vertices = [xc(:) yc(:) zc(:)];
                faces = zeros(N+2, N+1);
                for j = 1:N
                    faces(j, 1:4) = 2*(j-1) + [1 2 4 3];
                end
                faces(N+1, :) = 1 : 2 : size(vertices,1);
                faces(N+2, :) = 2 : 2 : size(vertices,1);
                faces(faces == 0) = NaN;
                h.joint(i) = patch('Faces', faces, 'Vertices', vertices);
            end
            
            if length(h.joint) == i
                %set(h.joint(i), 'erasemode', 'xor');
                set(h.joint(i), 'FaceColor', 0.3*[1 1 1], 'EdgeColor', 'none');

                % build a matrix of coordinates so we
                % can transform the cylinder in animate()
                % and hang it off the cylinder
                xyz = [xc(:)'; yc(:)'; zc(:)'; ones(1,length(xc(:)))]; 
                set(h.joint(i), 'UserData', xyz);
            end
        end
            
        if opt.jointaxis == 1
            % add a dashed line along the axis
            h.jointaxis(i) = line('xdata', [0;0], ...
                'ydata', [0;0], ...
                'zdata', [0;0], ...
                'color', 'blue', ...
                'linestyle', '--', ...
                'erasemode', 'xor');
        end
    end

%ANIMATE   move an existing graphical robot
%
%	animate(robot, q)
%
% Move the graphical robot to the pose specified by the joint coordinates q.
% Graphics are defined by the handle structure robot.handle.

function animate(robot, q)

	n = robot.n;
	h = robot.handle;
	L = robot.link;

	mag = h.mag;
    pr = robot.base;
    b = pr(1:3,4);
	%b = transl(robot.base);
	x = b(1);
	y = b(2);
	z = b(3);

	xs = b(1);
	ys = b(2);
	zs = h.zmin;

	% compute the link transforms, and record the origin of each frame
	% for the animation.
	t = robot.base;
	Tn = t;
	for j=1:(n-1),
		Tn(:,:,j) = t;

		t = t * L{j}(q(j));
        
        points = L{j}.points;
        for k = 1:size(points,2)
            p = t * [points(:,k); 1];
            x = [x; p(1)];
            y = [y; p(2)];
            z = [z; p(3)];
            xs = [xs; p(1)];
            ys = [ys; p(2)];
            zs = [zs; h.zmin];
        end
	end
	t = t *robot.tool; 
    x = [x; t(1,4)];
    y = [y; t(2,4)];
    z = [z; t(3,4)];
    xs = [xs; t(1,4)];
    ys = [ys; t(2,4)];
    zs = [zs; h.zmin];
    
    
    %%%%%%%%
    %Add prismatic gripper...
    %%%%%%%%%
    %The lines for the gripper are defined by four points.  A gripper
    %"elbow", or GE, and gripper tip, or GT
    d = q(6)/2+0.5;
    GEL = t*[d;0;0;1];
    GER = t*[-d;0;0;1];
    GTL = t*[d;0;1.125;1];
    GTR = t*[-d;0;1.125;1];
    
    x = [x; GEL(1);GTL(1);GEL(1);GER(1);GTR(1)];
    y = [y; GEL(2);GTL(2);GEL(2);GER(2);GTR(2)];
    z = [z; GEL(3);GTL(3);GEL(3);GER(3);GTR(3)];
    xs = [xs; GEL(1);GTL(1);GEL(1);GER(1);GTR(1)];
    ys = [ys; GEL(2);GTL(2);GEL(2);GER(2);GTR(2)];
%     zs = zeros(1,length(ys));
    zs = [zs; h.zmin;h.zmin;h.zmin;h.zmin;h.zmin];
    
	%
	% draw the robot stick figure and the shadow
	%
	set(h.robot, 'xdata', x, 'ydata', y, 'zdata', z);
    if isfield(h, 'shadow')
        set(h.shadow, 'xdata', xs, 'ydata', ys, 'zdata', zs);
    end
    
    if isfield(h, 'tool')
        xyz = get(h.tool, 'UserData');
        p = t*xyz;
        set(h.tool, 'XData', p(1,:),  'YData', p(2,:),  'ZData', p(3,:));
    end

	%
	% display the joints as cylinders with rotation axes
	%
    for j=1:(n-1),
        if isfield(h, 'joint'),
            wrist = robot.sphericalwrist;
            if any(j == wrist)
                if j == wrist(2)
                    % get coordinate data from the cylinder
                    xyz = get(h.joint(j), 'UserData');
                    xyz = Tn(:,:,j) * xyz;
                    
                    ncols = sqrt(size(xyz,2));
                    xc = reshape(xyz(1,:), [], ncols);
                    yc = reshape(xyz(2,:), [], ncols);
                    zc = reshape(xyz(3,:), [], ncols);
                    set(h.joint(j), 'Xdata', xc, 'Ydata', yc, 'Zdata', zc);
                end
            else
                % get coordinate data from the cylinder
                xyz = get(h.joint(j), 'UserData');
                xyz = Tn(:,:,j) * xyz;
                set(h.joint(j), 'Vertices', xyz(1:3, :).');
            end
			
        end

        if isfield(h, 'jointaxis')
            xyz_line = [0 0; 0 0; -1*mag 1*mag; 1 1];
            actuatoroffset = L{j}.actuatoroffset;
            xyz_line = bsxfun(@plus, xyz_line, [actuatoroffset; 0]);
            xyzl = Tn(:,:,j) * xyz_line;
            set(h.jointaxis(j), 'Xdata', xyzl(1,:), ...
                'Ydata', xyzl(2,:), ...
                'Zdata', xyzl(3,:));
        end
    end
    
    if isfield(h, 'tool')
        xyz = get(h.tool, 'UserData');
        xyz = t * xyz;
        
        set(h.tool, 'XData', xyz(1,:), 'YData', xyz(2,:), 'ZData', xyz(3,:));
    end

	drawnow
