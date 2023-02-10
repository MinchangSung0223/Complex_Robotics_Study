function drawRobot(l,q,base,v,vT,dt)
    eef_pos = [];
    for i = 1:1:length(l)
        li = l{i};
        qi = q{i};
        vi = v{i};
        basei = base{i};
        
        px1 = li(1)*cos(qi(1))+basei;
        py1 = li(1)*sin(qi(1));
        px2 = li(1)*cos(qi(1))+li(2)*cos(qi(1)+qi(2))+basei;
        py2 = li(1)*sin(qi(1))+li(2)*sin(qi(1)+qi(2));
        
        plot(basei,0,'ko');hold on;
        plot(px1,py1,'ko')
        plot(px2,py2,'ko')
        quiver([px2],[py2],[vi(1)],[vi(2)],0,'r','LineWidth',2,"MaxHeadSize",500)
%         annotation('arrow',[px2,px2+vi(1)*dt],[py2,py2+vi(2)*dt])
        plot([basei,px1,px2],[0,py1,py2],'k-')
        eef_pos = [eef_pos; px2, py2];
    end
    vT1 = vT{1};
    vT2 = vT{2};
    quiver([1],[1],[vT1(1)],[vT1(2)],0,'r','LineWidth',2,"MaxHeadSize",500)
    quiver([1],[1],[vT2(1)],[vT2(2)],0,'r','LineWidth',2,"MaxHeadSize",500)
    plot(eef_pos(:,1),eef_pos(:,2),'k:')
    drawCircle([1,1],0.5)
end
function drawCircle(center,radius)
    theta = 0:0.01:2*pi;
    x = radius*cos(theta)+center(1);
    y = radius*sin(theta)+center(2);
    plot(x,y,"k-");
end