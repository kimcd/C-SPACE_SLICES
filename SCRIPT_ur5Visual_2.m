%% SCRIPT_ur5Visual
%
% TODO
% points z-axis aren't aligned with nozzle z-axis
% clean the code up
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
H_nozzle_W = Tz(500) * Ty(-300); %* Rx(pi/2); % arbitrary. looks good enough
Scale = 150;
LineWidth = 3;
frameIDs = {'0','1','2','3','4','5','6','E','T','N'};
AxisLabels{1} = sprintf('x_%s',frameIDs{10});
AxisLabels{2} = sprintf('y_%s',frameIDs{10});
AxisLabels{3} = sprintf('z_%s',frameIDs{10});
nozzle_csys = triad(...
    'Parent',simObj.Axes,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',H_nozzle_W,'AxisLabels',AxisLabels);

%% Circle
% The goal here is to place each point on that circle under the nozzle
n=100;
s = linspace(0,2*pi,n);
points_T = [];
r=40;

for i = 1:n
    points_T(1,i) = r*sin(s(i)); % x-position
    points_T(2,i) = r*cos(s(i)); % y-position
    points_T(3,i) = 50; %z-position
    points_T(4,i) = 1; 
end

%t = hgtransform('Parent',simObj.Axes);
%set(X, 'Parent', t);
%points_T = Ry(pi/2)*points_T; % lift it up 
points_W =  simObj.ToolPose * points_T; %place circle at end of tool
plt_Waypoints = plot3(simObj.Axes,points_W(1,:),points_W(2,:),points_W(3,:),'.m');

%% Move tool to nozzle (single point)
for i = 1:n
    delete(plt_Waypoints) % 
    point_i_T = points_T(:,i);
    point_i_T = round(point_i_T,3);
    %compute the points, rotation about each tool frame axis (z-axis of point
    %is directed radially outward)
    r_x = atan2(point_i_T(3),point_i_T(2)); % x-rotation about tool frame

    
    H_point_i_T = Rx(r_x) * Ry(0) * Rz(0); % first-point w.r.t. tool frame
    H_point_i_T(:,4) = point_i_T;
    
    % H_desired_tool_pose = H_nozzle_0 * invSE(H_point_toolframe);
    % the desired tool pose is the transformation of the nozzle (w.r.t. world)
    % multiplied by the inverse of the transformation of the point (w.r.t. tool
    % frame).
    
    H_tool_W_desired = H_nozzle_W * invSE(H_point_i_T);
    simObj.ToolPose = H_tool_W_desired;
    H_point_i_W_new = simObj.ToolPose * H_point_i_T; 
    AxisLabels{1} = sprintf('x_{%d}',i);
    AxisLabels{2} = sprintf('y_{%d}',i);
    AxisLabels{3} = sprintf('z_{%d}',i);
    point_csys = triad(...
        'Parent',simObj.Axes,'Scale',Scale,'LineWidth',LineWidth,...
        'Matrix',H_point_i_W_new,'AxisLabels',AxisLabels);
    X_new = simObj.ToolPose * points_T;
    plt_Waypoints = plot3(simObj.Axes,X_new(1,:),X_new(2,:),X_new(3,:),'.m');
    drawnow;
    pause(.15);
    delete(point_csys);
end