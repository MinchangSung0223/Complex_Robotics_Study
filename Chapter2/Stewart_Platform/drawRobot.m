function drawRobot(ax,p_list,connection_list,M,Slist,q)
    Tj_pos_list=[];
    T1_pos_list=[];
    drawAxis(eye(4),0.1 ,1,eye(4),1,"",ax,0);
    for i = 1:1:length(M)
        Mi = M{i};
        Slisti = Slist{i};
        qi = q{i};
        Ti = FKinSpace(Mi,Slisti,qi);
        
        drawAxis(Ti,0.1 ,1,eye(4),1,"",ax,0);
        [T1,Tj]=drawLeg(ax,p_list{i},Slisti,qi);
        Tj_pos_list = [Tj_pos_list,Tj(1:3,4)];
        T1_pos_list = [T1_pos_list,T1(1:3,4)];
        plot3([T1(1,4), Tj(1,4)],[T1(2,4), Tj(2,4)],[T1(3,4), Tj(3,4)],'k--')
    end
    Tj_pos_list = [Tj_pos_list,Tj_pos_list(:,1)];
    T1_pos_list = [T1_pos_list,T1_pos_list(:,1)];
    plot3(Tj_pos_list(1,:),Tj_pos_list(2,:),Tj_pos_list(3,:))
    fill3(Tj_pos_list(1,:),Tj_pos_list(2,:),Tj_pos_list(3,:),'k',FaceAlpha=0.1)
    plot3(T1_pos_list(1,:),T1_pos_list(2,:),T1_pos_list(3,:))
    fill3(T1_pos_list(1,:),T1_pos_list(2,:),T1_pos_list(3,:),'k',FaceAlpha=0.1)
end
function [T1,Tj]=drawLeg(ax,p_list,Slist,q)
    expList={};
    Tj=eye(4);
    T1 = eye(4);
    pos_list = [];
    for j =1:1:length(q)
        Mj = eye(4);
        Mj(1:3,4) = p_list(j,:)';
        
        Tj = FKinSpace(Mj,Slist(:,1:j),q(1:j));
        if j==1
            T1 = Tj;
        end 
        
        if j==4
            continue;
        end
        drawAxis(Tj,0.1 ,1,eye(4),1,"",ax,0);
    end
   

end