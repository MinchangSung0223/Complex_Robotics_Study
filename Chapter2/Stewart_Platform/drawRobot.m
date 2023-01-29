function drawRobot(ax,q_a,q_p,LEG_NUM,Leg_Slist,Leg_M)
    n_a = length(q_a);
    n_p = length(q_p);
    q =zeros(n_a+n_p,1);
    count_a = 1;
    count_p = 1;
    for j =1:1:(n_a+n_p)
        if mod(j,7)==4
            q(j) = q_a(count_a);
            count_a = count_a+1;
        else
            q(j) = q_p(count_p);
            count_p = count_p+1;
        end
    end
    q_temp = q;
    q={};
    FK_list={};
    for i = 1:1:LEG_NUM
        q{i} = q_temp((i-1)*LEG_NUM+1:(i-1)*LEG_NUM+7);
        FK = FKinSpace(Leg_M{i},Leg_Slist{i},q{i});
        FK_list{i} = FK;
        drawAxis(FK,1 ,1,eye(4),1,"",ax,0)
    end

    


end