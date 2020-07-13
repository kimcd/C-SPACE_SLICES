clear all; close all; 
%%
% visualize individual point frames (outward)
fig = figure(1); 
axs = axes('Parent',fig); 

%% base_csys
Scale = 10;
LineWidth = 2;
frameIDs = {'W','p_1','p_2'};
AxisLabels{1} = sprintf('x_%s',frameIDs{1});
AxisLabels{2} = sprintf('y_%s',frameIDs{1});
AxisLabels{3} = sprintf('z_%s',frameIDs{1});
axis(axs,[-100 100 -100 100]); 
daspect(axs,[1 1 1])
base_csys = triad(... 
    'Parent',axs,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix', eye(4), 'AxisLabels',AxisLabels); 
%% circle
n = 30; % number of points to define circle
s = linspace(0,2*pi,n); % parameterization of circle
r=50; % radius
points_W = [];

for i = 1:n
    points_W(1,i) = r*cos(s(i)); % x-position
    points_W(2,i) = r*sin(s(i)); % y-position
    points_W(3,i) = 0;
    points_W(4,i) = 1;
    plot(axs,points_W(1,i),points_W(2,i),'.k'); 
end

%% csys for each point
Scale = 5;
LineWidth = .5;
for i = 1:n
    point_i_W = points_W(:,i);
    r_x = 0; 
    r_y = 0; 
    r_z = atan2(point_i_W(2),point_i_W(1)); 
    H_point_i_world = Rx(r_x)*Ry(r_y)*Rz(r_z); 
    H_point_i_world(1:3,4) = [point_i_W(1);point_i_W(2);point_i_W(3)]; 
    AxisLabels{1} = sprintf('x_%d',i);
    AxisLabels{2} = sprintf('y_%d',i);
    AxisLabels{3} = sprintf('z_%d',i);
    point_i_csys = triad(...
        'Parent',axs,'Scale',Scale,'LineWidth',LineWidth,...
        'Matrix',H_point_i_world,'AxisLabels',AxisLabels);
end