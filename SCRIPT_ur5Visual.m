%% SCRIPT_ur5Visual
% 
%
% C. KIM, 5JULY2020, JHU/APL

clearvars -EXCEPT hwObj
close all
clc

%% Initialize simulation
% -> This code has been tested on the UR5 and UR10 systems. Minor
% adjustments may be required to use it with the UR3.

if ~exist('simObj')
    % Create object
    simObj = URsim;
    % Initialize simulation
    simObj.Initialize;
    % Set a tool frame offset (e.g. for Robotiq end-effector not shown in
    % visualization)
    simObj.FrameT = Tz(160);
    
    grid(simObj.Axes,'on');

    % Hide frames
    frames = '0123456E';
    for i = 1:numel(frames)
        hideTriad(simObj.(sprintf('hFrame%s',frames(i))));
    end
end
%% Set nozzle in environment
H_nozzle = Tz(300) * Ty(-300); % arbitrary. looks good enough
Scale = 150;
LineWidth = 3;
frameIDs = {'0','1','2','3','4','5','6','E','T','N'};
AxisLabels{1} = sprintf('x_%s',frameIDs{10});
AxisLabels{2} = sprintf('y_%s',frameIDs{10});
AxisLabels{3} = sprintf('z_%s',frameIDs{10});
nozzle_csys = triad(...
    'Parent',simObj.Axes,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',H_nozzle,'AxisLabels',AxisLabels);

%% Circle
% The goal here is to place each point on that circle under the nozzle
s = linspace(0,2*pi,20);
X = [];
r=50;

for i = 1:20
    X(1,i) = r*cos(s(i)); % x-position
    X(2,i) = r*sin(s(i)); % y-position
    X(3,i) = 0; 
    X(4,i) = 1; 
end

%t = hgtransform('Parent',simObj.Axes); 
%set(X, 'Parent', t); 

X =  simObj.ToolPose * Ry(pi/2) * X; %place circle at end of tool
plt_Waypoints = plot3(simObj.Axes,X(1,:),X(2,:),X(3,:),'.k');

%% Move tool to nozzle (single point)
first_point_w = X(:,11) % first point w.r.t. world frame
first_point_tool = invSE(simObj.ToolPose) * first_point_w; % first point w.r.t. tool frame
first_point_tool = round(first_point_tool,3) 
%compute the points, rotation about each tool frame axis (z-axis of point
%is directed radially outward)
r_x = atan2(first_point_tool(2,1),first_point_tool(3,1)); % x-rotation about tool frame 
r_y = atan2(first_point_tool(1,1),first_point_tool(3,1)); % y-rotation about tool frame
r_z = atan2(first_point_tool(2,1),first_point_tool(1,1)); % z-rotation about tool frame

H_first_point_tool = Rx(r_x) * Ry(r_y) * Rz(r_z); % first-point w.r.t. tool frame
H_first_point_tool(:,4) = first_point_tool 

% H_desired_tool_pose = H_nozzle_0 * invSE(H_point_toolframe); 
% the desired tool pose is the transformation of the nozzle (w.r.t. world)
% multiplied by the inverse of the transformation of the point (w.r.t. tool
% frame). 

H_tool_desired = H_nozzle * invSE(H_first_point_tool)
simObj.ToolPose = H_tool_desired; 

X_new = H_tool_desired * X; 
plt_Waypoints = plot3(simObj.Axes,X_new(1,:),X_new(2,:),X_new(3,:),'.k');

%% Move tool to nozzle (animation)
for i = 1:size(X,2)
    point_w = X(:,1); % first point w.r.t. world frame
    point_tool = invSE(simObj.ToolPose) * point_w; % first point w.r.t. tool frame
    point_tool = round(point_tool,3);
    %compute the points, rotation about each tool frame axis (z-axis of point
    %is directed radially outward)
    r_x = atan2(point_tool(2,1),point_tool(3,1)); % x-rotation about tool frame
    r_y = atan2(point_tool(1,1),point_tool(3,1)); % y-rotation about tool frame
    r_z = atan2(point_tool(2,1),point_tool(1,1)); % z-rotation about tool frame
    
    H_point_tool = Rx(r_x) * Ry(r_y) * Rz(r_z); % first-point w.r.t. tool frame
    H_point_tool(:,4) = point_tool;
    
    % H_desired_tool_pose = H_nozzle_0 * invSE(H_point_toolframe);
    % the desired tool pose is the transformation of the nozzle (w.r.t. world)
    % multiplied by the inverse of the transformation of the point (w.r.t. tool
    % frame).
    
    H_tool_desired = H_nozzle * invSE(H_point_tool);
    simObj.ToolPose = H_tool_desired;
    drawnow;
    pause(.15);
end

%% BELOW IS NOT PART OF MAIN SCRIPT
%% Sphere
s = linspace(0,1,21);
X = [];
r=50;
for j= 2:20
    [x,y,z] = sphere;
    X(1,:) = r*x(j,:); % x-position
    X(2,:) = r*z(j,:)-250; % y-position
    X(3,:) = r*y(j,:); % z-position
    X(4,:) = 1;             % Append 1 (homogeneous coordinate)
    %plt_Waypoints = plot3(simObj.Axes,X(1,:),X(2,:),X(3,:),'.r');
end
X = simObj.ToolPose * X; 
plt_Waypoints = plot3(simObj.Axes,X(1,:),X(2,:),X(3,:),'.k');


%%

% Given some point on a circle, get the 
%H = simbObj.ToolPose; 
simObj.ToolPose = H_nozzle;
r=50;


%% Create path
% -> NOTE: This is provided for demonstration only. Perfect execution of
% this path will require infinite accelerations at the initial and final
% waypoints meaning the robot will not track perfectly.

%P = imread('anchor.png');
%M= model(P);

% Define dependent variable
s = linspace(0,1,21);
% Define circle radius (mm)
% Define position data
X = [];
r=50;

for j= 2:20
    [x,y,z] = sphere;
    X(1,:) = r*x(j,:); % x-position
    X(2,:) = r*z(j,:)-250; % y-position
    X(3,:) = r*y(j,:); % z-position
    X(4,:) = 1;             % Append 1 (homogeneous coordinate)
    
    % Transform coordinates into the workspace of the robot
    %pre-multiply steps are done from right to left in global frame
    X = Tz(500)*Rx(pi/2)* Tz(500)*X;
    
    % Plot waypoints
    plt_Waypoints = plot3(simObj.Axes,X(1,:),X(2,:),X(3,:),'.r');
    
    
    %% Animate simulation and move the robot to test
    % Home simulation
    simObj.Home;
    
    % Allow plot to update
    drawnow
    
    % Move through waypoints
    for i = 1:numel(s)
        % Define pose from waypoint
        H_cur = Tx(X(1,i))*Ty(X(2,i))*Tz(X(3,i))*Rx(pi/2);
        
        % Set simulation toolpose to waypoint pose
        simObj.ToolPose = H_cur;
        
        % Move robot to match simulation
        q = simObj.Joints;
               
        % Allow plot to update
        drawnow;
        pause(.15);
    end
end
