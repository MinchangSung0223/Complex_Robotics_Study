clear
L1 = 1;L2 =1;L3 = 1;
Lque = 0;

Slist = [ 0 0 1 0 0 0 ;
          0 0 1 0 -L1 0 ;
          0 0 1 0 -L1-L2 0 ;
          0 0 1 0 -L1-L2-L3 0 ]'
M = [1 0 0 L1+L2+L3+Lque; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
plist = [L1,0,0; L1+L2 0 0; L1+L2+L3 0 0 ; L1+L2+L3+Lque 0 0]
q = [45 -90 45,0]'./180.*pi;
q(4) = (2*pi-(q(1) +q(2) +q(3)))
Blist = Adjoint(TransInv(M))*Slist;
Jb = JacobianBody(Blist,q);
Jb = Jb([4,5],:);
f = figure(1);
f.Position = [680         177        1059         801];
for brace_x = 1:-0.1:0
    J2 = [1 -0.25;
          0 brace_x];
    A_2 = [1 0 0; 0 1 -0.25 ; 0 0 1]
    cla
    drawRobot(M,Slist,plist,q);
    T = FKinSpace(M,Slist,q);
    plot([T(1,4)-brace_x,T(1,4)-brace_x],[-0.25,0],'ks-')
    % [V,qdot_list]=drawJacobian(M,Slist,q,J2*Jb,"k")
    nullJ2 = eye(2)-pinv(J2)*J2;
    [U,SIGMA,V]=eig(Jb(1:2,1:3)*Jb(1:2,1:3)')
    u1 = U(:,1);
    u2 = U(:,2);
    v1 = V(1,:);
    v2 = V(2,:);
    
     V_b=drawJacobian(T(1,4),T(2,4),Jb);
%     [V,qdot_list]=drawJacobian(M,Slist,q,[J2,[0;0]],"b");
%     [V_b,qdot_list]=drawJacobian(M,Slist,q,J2,"g");
    drawJacobian_(T(1,4),T(2,4),J2,"y");
%     drawJacobian_(T(1,4),T(2,4),J2*Jb,"b");
    drawJacobian_(T(1,4),T(2,4),[Jb(1:2,1:3),[J2(1,2);0]],"blue");
%     drawJacobian_(T(1,4),T(2,4),pinv([Jb(1,1:3)-0.25*Jb(1,1:3),J2(1,2); 0 0 0 J2(2,2)])',"red");
    
    text(4,-2,"brace\_x : "+string(brace_x))
%     drawJacobian_(T(1,4),T(2,4),[Jb(1:2,1:3), [0;0] ; [0 0 0 -0.25; 0 0 0 brace_x]],"k");
    legend('','','','','','','','','','','J1','invJ1T','J2','J2J1')
    daspect([1,1,1])
    axis([-2 7 -5 5])
    drawnow;
end
% grid on;
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
    qdot = randn(n,1000);
    qdot2 = qdot.^2;
    sumqdot2 = sum(qdot2,1);
    qdot = qdot./sqrt(sumqdot2);
    V = J*qdot;
    F = pinv(J)'*qdot;
    for j = 1:2:n-1
        try
            Vj = V(j,:)'+ x;
            Vj_1 = V(j+1,:)'+ y;
            Vk = convhull(Vj,Vj_1);
            fill(Vj(Vk),Vj_1(Vk),'cyan','FaceAlpha',0.1,'LineStyle','--');
        catch
        end
    end

    Fx = F(1,:)'+ x;
    Fy = F(2,:)'+ y;
    Fk = convhull(Fx,Fy);
    fill(Fx(Fk),Fy(Fk),'magenta','FaceAlpha',0.1,'LineStyle','--');
end
function [V,qdot_list]=drawJacobian_(x,y,J,color_str)
    V = [];
    F = [];
    V_ = [];
    [m,n]=size(J);
    
    if n>3
        sample_num = 10000;
    else
        sample_num = 10000;
    end
    qdot = rand(n,sample_num)-0.5;
    for i =1:1:sample_num
        qdot(:,i)=qdot(:,i)./norm(qdot(:,i));
    end
    V = J*qdot;
    try
        Vx = V(1,:)'+x;
        Vy = V(2,:)'+y;
        Vk = convhull(Vx,Vy);
        fill(Vx(Vk),Vy(Vk),color_str,'FaceAlpha',0.1,'LineStyle','--');
    catch
        
    end
    try
        Vx2 = V(3,:)'+x;
        Vy2 = V(4,:)'+y;    
        Vk2 = convhull(Vx2,Vy2);
        fill(Vx2(Vk2),Vy2(Vk2),color_str,'FaceAlpha',0.1,'LineStyle','--');
    catch
        
    end

    
    
  
%      h=scatter(V(1,:)+x,V(2,:)+ y,color_str,'MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
%      h.SizeData = h.SizeData/10;
%      h1=scatter(V(3,:)+x,V(4,:)+ y,color_str,'MarkerFaceAlpha',1,'MarkerEdgeAlpha',1);
%      h1.SizeData = h.SizeData/10;
end