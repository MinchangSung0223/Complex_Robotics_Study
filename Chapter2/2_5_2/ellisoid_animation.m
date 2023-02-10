clear
L1 = 1;L2 =1;L3 = 1;
Lque = 2;

Slist = [ 0 0 1 0 0 0 ;
          0 0 1 0 -L1 0 ;
          0 0 1 0 -L1-L2 0 ;
          0 0 1 0 -L1-L2-L3 0 ]'
M = [1 0 0 L1+L2+L3+Lque; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
plist = [L1,0,0; L1+L2 0 0; L1+L2+L3 0 0 ; L1+L2+L3+Lque 0 0]



M_3 = [1 0 0 L1+L2+L3; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
q = [270 -30 -30,0]'./180.*pi;
f = figure(1);
f.Position = [680         177        1059         801];
for rot_z = (-pi/2):-0.01:(-pi/2-pi/3)
    
    Target = RpToTrans(eul2rotm([rot_z,0,0]),[-1.5 -2.5 0]')
    [q,ret]= IKinSpace(Slist(:,1:3), M_3, Target, q(1:3), 9999, 0.001)
    if ret==0
        continue;
    end
    q(4) = (2*pi-(q(1) +q(2) +q(3)))
    Blist = Adjoint(TransInv(M))*Slist;
    Jb = JacobianBody(Blist,q);
    Jb = Jb([4,5],:);
    cla;
    drawRobot(M,Slist,plist,q);
    V =drawJacobian(M,Slist,q,Jb)
    daspect([1,1,1])
    axis([-3 7 -5 5])
    grid on;
    drawnow;
end
function drawRobot(M,Slist,plist,q)
    prevTi = eye(4);
    plot(0,0,'ks','MarkerFaceColor',[0.0,0.0,0.0]);hold on;
    for i = 1:1:length(q)
        Mi = eye(4);
        Mi(1:3,4) = plist(i,:)';
        Ti = FKinSpace(Mi,Slist(:,1:i),q(1:i));
        
        plot(Ti(1,4),Ti(2,4),'ko')
        if i==4
            plot([prevTi(1,4) Ti(1,4)],[prevTi(2,4) Ti(2,4)],'b-')
        else
             plot([prevTi(1,4) Ti(1,4)],[prevTi(2,4) Ti(2,4)],'k-')
        end
        prevTi = Ti;
    end
end


function [V,qdot_list]=drawJacobian(M,Slist,q,J)
    T = FKinSpace(M,Slist,q);
    V = []
    V_ = []
    [X,Y,Z] = sphere(100)
    qdot_list = []
    for i = 1:1:length(X)
        for j = 1:1:length(X)
            qdot1 = X(i,j);
            qdot2 = Y(i,j);
            qdot3 = Z(i,j);
            qdot = [qdot1;qdot2;qdot3];
            V = [V,J(1:2,[1,2,3])*qdot];
            qdot_list = [qdot_list, qdot];
            V_ = [V_,eye(2,3)*qdot];
        end
    end
     h=scatter(V(1,:)+ T(1,4),V(2,:)+ T(2,4),'ko','MarkerFaceAlpha',0.1,'MarkerEdgeAlpha',0.1);
     h2=scatter(V_(1,:)+ T(1,4),V_(2,:)+ T(2,4),'ro','MarkerFaceAlpha',0.1,'MarkerEdgeAlpha',0.1);
     h.SizeData = h.SizeData/9;
     h2.SizeData = h2.SizeData/9;
end

