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
% plot in task space
q_square = build_square(17, [-1,-.5], .5, deg2rad(0));
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
q_animate = q_dwn_wrapped;
for i = 1:size(q_animate,2)
    %delete(cSpaceTracker)
    obj.Joints = q_animate(:,i); 
    cSpaceTracker=plot(mom_cspace,q_animate(1,i),q_animate(2,i),'ko');
    drawnow
    pause(.1)
    delete(cSpaceTracker)
end

%% BELOW NOT FUNCTIONAL
% %% TEST PLOTTING SIM
% qGoal = [0,.5];
% joints = LM_iKin2(L1, L2, qGoal); 
% obj.Joints = joints(:,2);
% drawnow
% 
% %% GIVEN COORDINATE, PLOT IN TASK SPACE AND C SPACE
% q_target 
% plot(mom,qObs(1),qObs(2),'rx');
% text(mom,qObs(1),qObs(2),'Obstacle')
% 
% %%
% %qGoal = [1.5;1]
% %qObs = [.7;.7];
% qObs = [0.5;0.5]; %obstacle
% 
% plot(mom,qObs(1),qObs(2),'rx');
% text(mom,qObs(1),qObs(2),'Obstacle')
% 
% poseCurr = obj.Frame0 * LM_fKin(1,1,obj.Joints); %find current tool pose
% qCurr = poseCurr(1:2,4); %current end-effector location
% 
% plot(mom,qCurr(1),qCurr(2),'gx')
% text(mom,qCurr(1),qCurr(2),'Initial EE Position')
% 
% trackLine = animatedline(mom);
% 
% % LINK1
% %  if link 1 can hit obstacle, plot c-obstacle
% if norm(qObs) <= L1
%     theta1 = atan2(qObs(2),qObs(1)); 
%     plot(axs2,[-pi pi],[-pi -pi],'k')
% end
% 
% %LINK2
% dL = linspace(0,1,100);
% dL = dL*obj.L2;
% cJoints = []; 
% theta_elbowup=[;]; 
% theta_elbowdwn=[;]; 
% j=0;
% cSpaceTracker=plot(axs2,0,0);
% 
% for i = 1:length(dL)
%     %cJoints(:,i) = LM_iKin(L1,dL(i),qObs); %configuration
%     L2=dL(i);
%         
%     D = (qObs(1)^2 + qObs(2)^2 - L1^2 - L2^2)/(2 * L1 * L2);
%     
%     if abs(D)<=1 && ~isinf(abs(D))
%         j=j+1; 
%         possConfig = true;
%         theta_elbowup(2,j) = atan2(-sqrt(1-D^2),D);
%         theta_elbowup(1,j) = atan2(qObs(2),qObs(1)) - atan2(L2*sin(theta_elbowup(2,j)),L1+L2*cos(theta_elbowup(2,j)));
%         
%         theta_elbowdwn(2,j) = atan2(sqrt(1-D^2),D);
%         theta_elbowdwn(1,j) = atan2(qObs(2),qObs(1)) - atan2(L2*sin(theta_elbowdwn(2,j)),L1+L2*cos(theta_elbowdwn(2,j)));
%     else
%         possConfig = false;
%     end
%     
% %     if possConfig
% %          plot(axs2,theta_elbowup(1,j),theta_elbowup(2,j),'r.');
% %          plot(axs2,theta_elbowdwn(1,j),theta_elbowdwn(2,j),'k.');
% %     end
%     
% end
% 
% %% ANIMATION
% for i = 1:size(theta_elbowup,2)-1
%     plot(axs2,[theta_elbowup(1,i),theta_elbowup(1,i+1)],[theta_elbowup(2,i) theta_elbowup(2,i+1)],'k')
% end
% 
% for i = 1:size(theta_elbowdwn,2)-1
%     plot(axs2,[theta_elbowdwn(1,i),theta_elbowdwn(1,i+1)],[theta_elbowdwn(2,i) theta_elbowdwn(2,i+1)],'k')
% end
% 
% for i = 1:size(theta_elbowup,2)
%     delete(cSpaceTracker)
%     obj.Joints = theta_elbowup(:,i); 
%     cSpaceTracker=plot(axs2,theta_elbowup(1,i),theta_elbowup(2,i),'ko');
%     drawnow
%     pause(.1)
% end
% 
% for i = 1:size(theta_elbowdwn,2)
%     delete(cSpaceTracker)
%     obj.Joints = theta_elbowdwn(:,i); 
%     cSpaceTracker=plot(axs2,theta_elbowdwn(1,i),theta_elbowdwn(2,i),'ko');
%     drawnow
%     pause(.1)
% end

