clear
L1 = 1;L2 =1;L3 = 1;
Slist = [ 0 0 1 0 0 0 ;
          0 0 1 0 -L1 0 ;
          0 0 1 0 -L1-L2 0 ]'
M = [1 0 0 L1+L2+L3; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
plist = [L1,0,0; L1+L2 0 0; L1+L2+L3 0 0 ]
q = [45 -90 45]'./180.*pi;
Blist = Adjoint(TransInv(M))*Slist;
Jb = JacobianBody(Blist,q);
Jb = Jb([4,5],:);
J2 = [-0.25 ; 0 ; 1];
A2 = [1 0 0 ; 0 1 -0.25; 0 0 1]
J2_ = pinv(A2)*J2;
H2T = [1 0 0]';
H2T_ = pinv(A2 )*H2T

A = [eye(2); eye(2)]
C1 = [0 0.3536 0 -0.125; 1.2071 0.8536 0.5 0.125];
C2 = [0 -0.5 0 -0.1768; -1.7071 -1.2071 -0.7071 0.1768]
invhalf_Ohm = eye(2);
Val = C1*null(C2')'*invhalf_Ohm
% 
f = figure(1);
f.Position = [680         177        1059         801];
drawRobot(M,Slist,plist,q);
T = FKinSpace(M,Slist,q);
V=drawJacobian(T(1,4),T(2,4),Jb);


% J = [0 0.7071 0 0; 2.4142 1.7071 1 0; 0 0 0 -0.25 ; 0 0 0 0.25]
% V=drawJacobian(T(1,4),T(2,4),J);
% V=drawJacobian(T(1,4),T(2,4),[1 -0.25; 0 0.25]);
daspect([1,1,1])
axis([-2 7 -5 5])
drawnow;
grid on;
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


function V=drawJacobian(x,y,J)
    V = [];
    F = [];
    [m,n]=size(J)
    qdot = randn(n,10000);
    qdot2 = qdot.^2;
    sumqdot2 = sum(qdot2,1);
    qdot = qdot./sqrt(sumqdot2);
    V = J*qdot;
    F = pinv(J)'*qdot;
    for j = 1:2:n-1
        Vj = V(j,:)'+ x;
        Vj_1 = V(j+1,:)'+ y;
        Vk = convhull(Vj,Vj_1);
        fill(Vj(Vk),Vj_1(Vk),'cyan','FaceAlpha',0.1,'LineStyle','--');
    end

    Fx = F(1,:)'+ x;
    Fy = F(2,:)'+ y;
    Fk = convhull(Fx,Fy);
    fill(Fx(Fk),Fy(Fk),'magenta','FaceAlpha',0.1,'LineStyle','--');
end

