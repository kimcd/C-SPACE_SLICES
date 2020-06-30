function joints = LM_iKin(L1,L2,qGoal)
%TODO
% need to do for both angles
%qGoal = [0.2;0.2];
theta = [;];

y=sqrt(1-((qGoal(1)^2+qGoal(2)^2-L1^2-L2^2)/(2*L1*L2))^2);

if ~isreal(y)
    theta(2,:)=nan;
    theta(1,:)=nan;
else    
    theta(2,:) = atan2(y,(qGoal(1)^2+qGoal(2)^2-L1^2-L2^2)/(2*L1*L2));
    theta(1,:) = atan2(qGoal(2),qGoal(1))-atan2(L2*sin(theta(2)),L1+L2*cos(theta(2)));
end

joints = [theta(1);theta(2)];
end