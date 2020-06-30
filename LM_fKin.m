function pose = LM_fKin(L1,L2,joints)

 x = L1*cos(joints(1))+L2*cos(joints(1)+joints(2));
 y = L1*sin(joints(1))+L2*sin(joints(1)+joints(2));
 
joint = joints(1)+joints(2);
pose = Rz(joint);
pose = Tx(x)*Ty(y)*pose;

end
