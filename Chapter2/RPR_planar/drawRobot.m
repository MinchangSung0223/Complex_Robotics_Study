function drawRobot(ax,M,Slist,q,p_list)

triangle_pos_list = []
for i=1:1:length(Slist)
    pos_list = []
    for j=1:1:length(q{i})
        Slist_i = Slist{i};
        q_i = q{i};
        p = p_list{i};
        Mj = eye(4);
        Mj(1:3,4) = p(j,:)';
        Tj = FKinSpace(Mj,Slist_i(:,1:j),q_i(1:j));
        pos_list =[pos_list ; Tj(1:3,4)'];
        if j == length(q{i})
            triangle_pos_list = [triangle_pos_list; Tj(1:3,4)'];
        end
        drawAxis(Tj,0.1,1,eye(4),1,"",ax,0);
    end
    Tb = FKinSpace(M{i},Slist{i},q{i});
    pos_list =[pos_list ; Tb(1:3,4)'];
    plot3(pos_list(:,1),pos_list(:,2),pos_list(:,3),'k--')
    drawAxis(Tb,0.1,1,eye(4),1,"",ax,0)
end
triangle_pos_list = [triangle_pos_list; triangle_pos_list(1,:)]
plot3(triangle_pos_list(:,1),triangle_pos_list(:,2),triangle_pos_list(:,3),'k',LineWidth=2)

end