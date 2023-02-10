L1 = 1 ;L2 = 1; L3 = 1;
Slist = [ 0 0 1 0  0  0;
          0 0 1 0 -L1 0;
          0 0 1 0 -L1-L2 0]';

pos_list = [0 0 0; L1 0 0 ; L1+L2 0 0  ]
M = [1 0 0 L1+L2+L3; 0 1 0 0; 0 0 1 0 ; 0 0 0 1];
q=[0 -90 -90]'./180.*pi
Blist = Adjoint(TransInv(M))*Slist;
Jb = JacobianBody(Blist,q)
Jb = Jb([4,5],:)
T = FKinSpace(M,Slist,q)
J2 = [1 -0.25; 0 0.25];


f = figure(1);
f.Position = [  680   53   1097  925];


drawRobot(M,Slist,pos_list,q);
drawJacobian(M,Slist,q,Jb,J2)



daspect([1,1,1])
% grid on;
daspect([1,1,1])
axis([-1 5 -3 3])

