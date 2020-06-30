classdef linkManipulator < matlab.mixin.SetGet % Handle
    
    properties(GetAccess='public', SetAccess='public')
        Figure      % Figure containing the simulation axes
        Axes        % Axes containing the simulation
    end %end properties
    
    properties(GetAccess='public', SetAccess='public')
        Joints      % 1x6 array containing joint values (radians)
        Pose        % 4x4 homogeneous transform representing the end-effector pose relative to the world frame
        ToolPose    % 4x4 homogeneous transform representing the tool pose relative to the world frame
    end % end properties
    
    properties(GetAccess='public', SetAccess='private')
        DHtable     % DH table associated with robot
        L1
        L2
    end % end properties

    properties(GetAccess='public', SetAccess='private', Hidden=true)
        Frame0 %frame 0 transformation matrix
        Frame1 %frame 1 transformation matrix
        Frame2 %frame 2 transformation matrix
        FrameE
        
        hFrame0 %frame 0 transformation object
        hFrame1
        hFrame2
        
        hLink1 %Link 1 transform object
        hLink2 %Link 2 transform object
        hLink3 %Link 3 transform object
        
        lLink1 %line object for link 1
        lLink2 %line object for link 2
        lLink3 %line object for link 3
        
        Joint1
        Joint2
        
        hJoint1 %joint object
        hJoint2   
    end % end properties
    
    properties(GetAccess='public', SetAccess='private', Hidden=true)
        Joint2Pos
        Joint3Pos
    end
        
    %Constructor / Destructor
    methods(Access = 'public')
        function obj = linkManipulator
            % Create linkManipulator Object
        end
    end %end methods
    
    % Initialization
    methods(Access='public')
        function Initialize(obj)
            Scale = .25;
            LineWidth = 1;
            fig = figure;
            axs = axes('Parent',fig);
            axis(axs,[-3 3 -3 3])
            set(fig,'Name',sprintf('Robot Visualization'),...
                'MenuBar','none','NumberTitle','off','ToolBar','Figure');
            daspect(axs,[1 1 1]);
            hold(axs,'on');
            obj.L1 = 1; 
            obj.L2 = 1; 
            %homogenous tranformation matrices
            obj.Frame0 = eye(4); %H^w_0
            obj.Frame1 = Tx(obj.L1)*Ty(0)*Tz(0)*Rz(0);%H^0_1
            obj.Frame2 = Tx(obj.L2)*Ty(0)*Tz(0)*Rz(0);%H^1_2
            %obj.FrameE = Tx(obj.L2)*Ty(0)*Tz(0)*Rz(0);%H^2_E
            
            %link hgtransform objects
            obj.hLink1 = hgtransform('Parent',axs);
            obj.hLink2 = hgtransform('Parent',axs);
            %obj.hLink3 = hgtransform('Parent',axs);
            
            obj.hFrame0 = triad(...
                'Parent',axs','Scale',Scale,'LineWidth',LineWidth,...
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
            xlabel(axs,'x [in]');
            ylabel(axs,'y [in]');
            obj.Figure = fig;
            obj.Axes = axs;
            
        end %end function
    end %end methods
    
    %General Use
    methods(Access='public') 
        function Home(obj) 
            joints = [pi/4;-pi/4];
            obj.Joints = joints; 
        end
        
    end %end methods
    
    
    % Getter and Setters
    methods
        function joints =get.Joints(obj)
             joints = obj.Joints;
        end
        
        function obj = set.Joints(obj,joints)
            if numel(joints)~=2
                error('Joint configuration must be specified as a 2-element array.');
            end
            joints = reshape(joints,2,1);
            for i = 1:numel(joints)
                eval(sprintf('g = obj.hJoint%d;',i));
                set(g,'Matrix',Rz(joints(i)));
            end
            obj.Joints = joints;
        end
        
        function joint = get.Joint1(obj)
            joints = obj.Joints; 
            joint = joints(1); 
        end
        
        function joint = get.Joint2(obj) 
            joints = obj.Joints; 
            joint = joints(2);
        end
        
        function pos = get.Joint2Pos(obj) %H^0_1
            joints = obj.Joints;
            x = obj.L1 * cos(joints(1)); 
            y = obj.L1 * sin(joints(1)); 
            pos = [x;y];
            %obj.Frame1 = frame; 
        end
        
        function pos = get.Joint3Pos(obj) %H^1_2
            joints = obj.Joints;
            x = obj.L1 * cos(joints(1)) + obj.L2 * cos(joints(1) + joints(2));
            y = obj.L1 * sin(joints(1)) + obj.L2 * sin(joints(1) + joints(2));
            pos = [x;y];
            %obj.Frame2 = frame;
        end
          
    end %end methods
    
    
end %end classdef



