clear all; close all
fig = figure(1);
sim_axs = subplot(2,1,1);  % link_sim is an axis handle

Scale = .25;
LineWidth = 1;

axis(sim_axs,[-3 3 -3 3])

set(fig,'Name',sprintf('Robot Visualization'),...
    'MenuBar','none','NumberTitle','off','ToolBar','Figure');

daspect(sim_axs,[1 1 1]);

%%
hold(sim_axs,'on');
obj.L1 = 1;  % link 1 length
obj.L2 = 1;  % link 2 length
%homogenous tranformation matrices
obj.Frame0 = eye(4); %H^w_0
obj.Frame1 = Tx(obj.L1)*Ty(0)*Tz(0)*Rz(0);%H^0_1
obj.Frame2 = Tx(obj.L2)*Ty(0)*Tz(0)*Rz(0);%H^1_2
%obj.FrameE = Tx(obj.L2)*Ty(0)*Tz(0)*Rz(0);%H^2_E

%link hgtransform objects
obj.hLink1 = hgtransform('Parent',sim_axs);
obj.hLink2 = hgtransform('Parent',sim_axs);
%obj.hLink3 = hgtransform('Parent',axs);

obj.hFrame0 = triad(...
    'Parent',sim_axs,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',obj.Frame0);
obj.hJoint1 = hgtransform('Parent',obj.hFrame0);
obj.lLink1 = line(obj.hJoint1,[0 obj.L1],[0 0],'LineWidth',2);
obj.hFrame1 = triad(...
    'Parent',obj.hJoint1,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',obj.Frame1);%,'AxisLabels',AxisLabels);

obj.hJoint2 = hgtransform('Parent',obj.hFrame1);
obj.lLink2 = line(obj.hJoint2,[0 obj.L2],[0 0],'LineWidth',2);
obj.hFrame2 = triad(...
    'Parent',obj.hJoint2,'Scale',Scale,'LineWidth',LineWidth,...
    'Matrix',obj.Frame2);%,'AxisLabels',AxisLabels);

obj.Joints = [0;0];

%Define axes labels
xlabel(sim_axs,'x [in]');
ylabel(sim_axs,'y [in]');

%%
obj.Figure = fig;
obj.Axes = axs;

c_space = subplot(2,1,2);