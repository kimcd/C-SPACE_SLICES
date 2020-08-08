%%
%SUMMARY
% Tx * Rx * Tz
% reading from pre-multiply (right to left), transformations are done
% w.r.t. the global axis. 
% OR
% reading from post-multiply (left to right), transofmration are done
% w.r.t. the local axis. 

%%
clear all; close all;
fig = figure; 
axs = axes('Parent',fig); 
view(3);
axis([-1 1 -1 1 -1 1])
grid(axs,'on'); 
Scale = .25;
LineWidth = 1;
FrameW = eye(4); 
world_ax = hgtransform('Parent',axs); 
hFrameW = triad(...
    'Parent',axs','Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',FrameW);

%%
Frame0 = eye(4); 
%ax_0 = hgtransform('Parent',axs);
hFrame0 = triad(...
    'Parent',axs','Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',Frame0);

%ax_1 = hgtransform('Parent',axs);
hFrame1 = triad(...
    'Parent',axs','Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',Frame0);

%% Pre-Multiply Frame 0 
% verbal description: 
% 1) start at frame 0, translate in global x-axis by 0.5 units
% 2) translate in global y-axis by 0.5 units
% 3) rotate about global x-axis by pi/2 radians
% which is same as: 
% 1) start at frame 0, rotate about local x-axis by pi/2 radians
% 2) translate in local y-axis by 0.5 units
% 3) translate in local x-axis by 0.5 units
Frame0 = Rx(pi/2)*Ty(.5)*Tx(.5);
set(hFrame0, 'Matrix', Frame0); 

%% Post-Multiply Frame 1
% verbal description: 
% 1) translate in local x-axis by 0.5 units
% 2) translate in local y-axis by 0.5 units
% 3) rotate about local x-axis by pi/2 radians
% which is same as: 
% 1) rotate about global x-axis by pi/2 radians
% 2) translate in global y-axis by 0.5 units
% 3) translate in global x-axis by 0.5 units
Frame1 = Tx(.5)*Ty(.5)*Rx(pi/2);
set(hFrame1,'Matrix',Frame1); 


