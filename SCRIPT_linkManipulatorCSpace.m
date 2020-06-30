%SCRIPT_linkManipulatorCSpace
%for points along the link (including endpoints) determine the
%configuration, if it exists, that allows that point to intersect the
%obstacle. those configurations, will produce the c-obstacles 

%TODO
%create for link 1

clear all
close all

%% initialize simulation
obj = linkManipulator;
obj.Initialize;
obj.Joints = [deg2rad(1);deg2rad(1)];
L1=obj.L1;
L2=obj.L2;

%qGoal = [1.5;1]
%qObs = [.7;.7];
qObs = [0.5;0.5];

mom=obj.Axes;

plot(mom,qObs(1),qObs(2),'rx');
text(mom,qObs(1),qObs(2),'Obstacle')

poseCurr = obj.Frame0 * LM_fKin(1,1,obj.Joints); %find current tool pose
qCurr = poseCurr(1:2,4); %current end-effector location

plot(mom,qCurr(1),qCurr(2),'gx')
text(mom,qCurr(1),qCurr(2),'Initial EE Position')

trackLine = animatedline(mom);

%% PLOT C-SPACE OBSTACLE
fig2=figure(2);
axs2=axes('Parent',fig2); 
hold(axs2,'on');
axis(axs2,[-pi pi -pi pi])
grid(axs2,'on')
xlabel(axs2,'-\pi < \theta_1 < +\pi'); ylabel(axs2,'-\pi < \theta_2 < +\pi');
title(axs2,'Configuration Space')

%LINK1
if norm(qObs) <= L1
    theta1 = atan2(qObs(2),qObs(1))
    plot(axs2,[theta1 theta1],[-pi pi],'k')
end

%LINK2
dL = linspace(0,1,100);
dL = dL*obj.L2;
cJoints = []; 
theta_elbowup=[;]; 
theta_elbowdwn=[;]; 
j=0;
cSpaceTracker=plot(axs2,0,0);

for i = 1:length(dL)
    %cJoints(:,i) = LM_iKin(L1,dL(i),qObs); %configuration
    L2=dL(i);
        
    D = (qObs(1)^2 + qObs(2)^2 - L1^2 - L2^2)/(2 * L1 * L2);
    
    if abs(D)<=1 && ~isinf(abs(D))
        j=j+1; 
        possConfig = true;
        theta_elbowup(2,j) = atan2(-sqrt(1-D^2),D);
        theta_elbowup(1,j) = atan2(qObs(2),qObs(1)) - atan2(L2*sin(theta_elbowup(2,j)),L1+L2*cos(theta_elbowup(2,j)));
        
        theta_elbowdwn(2,j) = atan2(sqrt(1-D^2),D);
        theta_elbowdwn(1,j) = atan2(qObs(2),qObs(1)) - atan2(L2*sin(theta_elbowdwn(2,j)),L1+L2*cos(theta_elbowdwn(2,j)));
    else
        possConfig = false;
    end
    
%     if possConfig
%          plot(axs2,theta_elbowup(1,j),theta_elbowup(2,j),'r.');
%          plot(axs2,theta_elbowdwn(1,j),theta_elbowdwn(2,j),'k.');
%     end
    
end

for i = 1:size(theta_elbowup,2)-1
    plot(axs2,[theta_elbowup(1,i),theta_elbowup(1,i+1)],[theta_elbowup(2,i) theta_elbowup(2,i+1)],'k')
end

for i = 1:size(theta_elbowdwn,2)-1
    plot(axs2,[theta_elbowdwn(1,i),theta_elbowdwn(1,i+1)],[theta_elbowdwn(2,i) theta_elbowdwn(2,i+1)],'k')
end

for i = 1:size(theta_elbowup,2)
    delete(cSpaceTracker)
    obj.Joints = theta_elbowup(:,i); 
    cSpaceTracker=plot(axs2,theta_elbowup(1,i),theta_elbowup(2,i),'ko');
    drawnow
    pause(.1)
end

for i = 1:size(theta_elbowdwn,2)
    delete(cSpaceTracker)
    obj.Joints = theta_elbowdwn(:,i); 
    cSpaceTracker=plot(axs2,theta_elbowdwn(1,i),theta_elbowdwn(2,i),'ko');
    drawnow
    pause(.1)
end

