function drawRobot(ax,M,Slist,p_list,q)
    arm_num = length(q);
    drawAxis(eye(4),0.25 ,1,eye(4),1,"\{0\}",ax,0)
    txt_list{1} = "Leef";
    txt_list{2} = "Reef";
    for i = 1:1:arm_num
        Slist_i = Slist{i};
        M_i = M{i};
        q_i = q{i};
        p_list_i  = p_list{i};
        Ti = FKinSpace(M_i,Slist_i,q_i);
        joint_num = length(q_i);
        pos_list = [0,0,0];
        for j =1:1:joint_num
            w_j = Slist_i(1:3,j);
            p_j = p_list_i(j,:);
            Mj = RpToTrans(MatrixExp3(VecToso3(w_j)*0),p_j');
            Tj = FKinSpace(Mj,Slist_i(:,1:j),q_i(1:j));
            pos_list = [pos_list; Tj(1:3,4)'];
%             drawAxis(Tj,0.25 ,1,eye(4),1,"",ax,0)
        end
        pos_list=[pos_list; Ti(1:3,4)'];
        plot3(pos_list(:,1),pos_list(:,2),pos_list(:,3),'k.',MarkerSize=20)
        plot3(pos_list(:,1),pos_list(:,2),pos_list(:,3),'k--',LineWidth=1)
        drawAxis(Ti,0.25 ,1,eye(4),1,txt_list{i},ax,0)
    end
end