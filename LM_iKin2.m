function joints = LM_iKin2(L1,L2,qGoal)
%TODO
% need to do for both angles
%qGoal = [0.2;0.2];
theta = [;];

D = (qGoal(1)^2 + qGoal(2)^2 - L1^2 - L2^2)/(2 * L1 * L2);

if abs(D)<=1 && ~isinf(abs(D))
    j=1;
    possConfig = true;
    theta_elbowup(2,j) = atan2(-sqrt(1-D^2),D);
    theta_elbowup(1,j) = atan2(qGoal(2),qGoal(1)) - atan2(L2*sin(theta_elbowup(2,j)),L1+L2*cos(theta_elbowup(2,j)));
    
    theta_elbowdwn(2,j) = atan2(sqrt(1-D^2),D);
    theta_elbowdwn(1,j) = atan2(qGoal(2),qGoal(1)) - atan2(L2*sin(theta_elbowdwn(2,j)),L1+L2*cos(theta_elbowdwn(2,j)));
    joints = [theta_elbowup, theta_elbowdwn];
else
    warning('Desired goal is outside of the robot workspace.')
    possConfig = false;
    
end

end