% SCRIPT_linkManipulatorCSpace
%  for points along the link (including endpoints) determine the
%  configuration, if it exists, that allows that point to intersect the
%  obstacle. those configurations, will produce the c-obstacles

% TODO
%  subplot config space
%  Configuration Jump & Collision.
%  q_square = build_square(17, [-.5,-.1], 1, deg2rad(15))
%  is configuration jump always mean collision?

clear all
close all

%% INITIALIZE VISUALIZATION
obj = linkManipulator;
obj.Initialize;
obj.Joints = [deg2rad(1); deg2rad(1)];  % set to 1 to avoid singularity
L1=obj.L1;
L2=obj.L2;
mom_sim = obj.simAxs;
mom_cspace = obj.cSpaceAxs;

%% PLOT A SQUARE IN TASK-SPACE & C-SPACE
n = 10; 
center = [.5,-1]; 
animate_flag = false; 
ang = linspace(0,90,n);
%n = 1;
%ang = 90; 
for i = 1:n
    % plot in task space
    q_square = build_square(17, center, .5, deg2rad(ang(i)));
    obj.plot_taskspace(q_square); % plot in taskspace
    
    % ikin
    q_up = zeros(2, size(q_square,2));
    q_dwn = zeros(2, size(q_square,2));
    for i = 1:size(q_square,2)
        q_joints = LM_iKin2(L1, L2, q_square(:,i));
        q_up(:,i) = q_joints(:,1);
        q_dwn(:,i) = q_joints(:,2);
    end
    q_up_wrapped = wrapToPi(q_up);
    q_dwn_wrapped = wrapToPi(q_dwn);
    
    %plot in c space
    obj.plot_cspace(q_up_wrapped, 'rx');
    obj.plot_cspace(q_dwn_wrapped, 'kx');
    
    %% ANIMATE
    if animate_flag
        q_animate = q_dwn_wrapped;
        for i = 1:size(q_animate,2)
            %delete(cSpaceTracker)
            obj.Joints = q_animate(:,i);
            cSpaceTracker=plot(mom_cspace,q_animate(1,i),q_animate(2,i),'ko');
            drawnow
            pause(.1)
            delete(cSpaceTracker)
        end
    end
    
end
