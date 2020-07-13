%% SCRIPT_settingPrintFrames
% This is a demo script to demonstrate how local coordinate frames are 
% assigned to points that lie on a sphere. It was used to create the
% POINT_FRAME function
%
% Key Constraints: 
% Local Z-axis is directed radially outward from the center of the sphere
% through the point.
% Local X-axis is directed toward the direction of the next point in the
% chain. The direction is determined by projecting the next point onto the
% plane that is normal to the local Z-axis. 


%%
clear all; close all 

%%
fig = figure; 
axs = axes('Parent',fig); 
view(axs,3); 
grid(axs,'on'); 
xlabel(axs,'x [mm]'); ylabel(axs,'y [mm]'); zlabel(axs,'z [mm]');
axis(axs, [-1000 1000 -1000 1000 -200 1000])
daspect(axs, [ 1 1 1]);

%% Tool Frame
Scale = 150;
LineWidth = 3;
H_tool2W = [1 0 0 -817.25; 0 0 -1 -351.45; 0 1 0 -5.191; 0 0 0 1];

AxisLabels{1} = sprintf('x_T');
AxisLabels{2} = sprintf('y_T');
AxisLabels{3} = sprintf('z_T');

tool_csys = triad(...
    'Parent',axs,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',H_tool2W,'AxisLabels',AxisLabels);

%% Circle points
n=15;
s = linspace(0,2*pi,n);
points_T = [];
r=80;

for i = 1:n
    points_T(1,i) = r*sin(s(i)); % x-position
    points_T(2,i) = r*cos(s(i)); % y-position
    points_T(3,i) = 50; %z-position
    points_T(4,i) = 1; 
end

points_W =  H_tool2W * points_T; %place circle at end of tool
plt_circle = plot3(axs,points_W(1,:),points_W(2,:),points_W(3,:),'.m');

%% Local Frames
for i = 1:n
    z_dist = points_W(1:3,i) - H_tool2W(1:3,4);
    z = z_dist/norm(z_dist); % z-unit vector 
    line_plt = plot3(axs,...
        [H_tool2W(1,4) points_W(1,i)],...
        [H_tool2W(2,4) points_W(2,i)],...
        [H_tool2W(3,4) points_W(3,i)],...
        'b');
    % PROJECTION OF POINT ONTO PLANE
    orig = points_W(1:3,i); % origin = point of interest
    nxt_i = mod(i,n)+1;
    nxt_point = points_W(1:3,nxt_i); % next point determines x-axis direction
    v = nxt_point - orig; % vector between point and next point
    dist = dot(z,v); % projection of vector onto z-unit vector 
    projected_point = nxt_point - dist*z; % projection of point onto plane
    plot3(projected_point(1),projected_point(2),projected_point(3),'kx');
    x_dist = projected_point - orig;
    x = x_dist/norm(x_dist); % x-unit vector
    y = cross(z,x); % y-unit vector
    scale = 100;
    z_plt = plot3(axs,...
        [0 scale*z(1)],...
        [0 scale*z(2)],...
        [0 scale*z(3)],...
        'b');
    x_plt = plot3(axs,...
        [0 scale*x(1)],...
        [0 scale*x(2)],...
        [0 scale*x(3)],...
        'r');
    y_plt = plot3(axs,...
        [0 scale*y(1)],...
        [0 scale*y(2)],...
        [0 scale*y(3)],...
        'g');
    % CONSTRUCTING ROTATION MATRIX FROM UNIT VECTORS
    H_point2W = [x y z [orig(1); orig(2); orig(3)]; [0 0 0 1]];
    point_csys = triad(...
    'Parent',axs,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',H_point2W,'AxisLabels',AxisLabels);
    pause(2)
    delete(point_csys); 
end

%%
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