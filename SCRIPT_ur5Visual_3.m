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
Scale = 50;
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
    i
    delete(plt_Waypoints)
    
    H_point2W = point_frame(simObj.ToolPose, points_T(1:3,i), points_T(1:3,i+1));
    
    H_point2T = invSE(simObj.ToolPose) * H_point2W; 
    
    H_tool2W_desired = Tz(-100)*(H_nozzle_W * invSE(H_point2T)); % lower the nozzle tip by 100mm so axes don't overlap
    simObj.ToolPose = H_tool2W_desired;
    H_point2W_new = simObj.ToolPose * H_point2T; 
    
    AxisLabels{1} = sprintf('x_{%d}',i);
    AxisLabels{2} = sprintf('y_{%d}',i);
    AxisLabels{3} = sprintf('z_{%d}',i);
    
    Scale = 50;
    
    point_csys = triad(...
        'Parent',simObj.Axes,'Scale',Scale,'LineWidth',LineWidth,...
        'Matrix',H_point2W_new,'AxisLabels',AxisLabels);
    %X_new = simObj.ToolPose * points_T;
    %plt_Waypoints = plot3(simObj.Axes,X_new(1,:),X_new(2,:),X_new(3,:),'.m');
    drawnow;
    pause(.15);
    delete(point_csys);
end

%%
    q_point2sphere = points_T(1:3,i)
    q_nxtpoint2sphere = points_T(1:3,i+1)
    H_sphere2W = simObj.ToolPose % invertible
    
    q_point2sphere = reshape(q_point2sphere,3,1);
    q_nxtpoint2sphere = reshape(q_nxtpoint2sphere,3,1); 
    
    q_point2W = H_sphere2W * [q_point2sphere; 1]; 
    q_nxtpoint2W = H_sphere2W * [q_nxtpoint2sphere;1]; 
    
    q_point2W = q_point2W(1:3);
    q_nxtpoint2W = q_nxtpoint2W(1:3);
    
    z_unit = (q_point2W - H_sphere2W(1:3,4))/...
        norm(q_point2W - H_sphere2W(1:3,4)); %z-axis unit normal radially outward from sphere center through point
    
    dist_v = q_nxtpoint2W - q_point2W; % vector from point to next point
    proj_dist_v = dot(z_unit, dist_v); % length of projected vector onto unit norm
    
    projected_point2W = q_nxtpoint2W - proj_dist_v * z_unit; % projection of point onto the plane
    
    x_unit = (projected_point2W - q_point2W)/...
        norm(projected_point2W - q_point2W); % x-axis unit normal
    
    y_unit = cross(z_unit, x_unit); 

    H_point2W = [x_unit y_unit z_unit...
        [q_point2W(1); q_point2W(2);...
        q_point2W(3)]; [0 0 0 1]]; 

