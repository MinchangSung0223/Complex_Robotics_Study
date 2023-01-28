function drawRobot(M,Slist,thetalist,T0,p_list,txt,base_txt,ax)
    hold on;
    n = length(thetalist);
    exp_list= {};
    Mlist = {};
    for i = 1:1:n
         S_i = Slist(:,i);
         w = S_i(1:3);
         theta_i = thetalist(i);
         M_ = eye(4);
         M_(1:3,4) = p_list(i,:)';

         exp_ = MatrixExp6(VecTose3(S_i)*theta_i);
         exp_list{end+1} = exp_;
         Mlist{end+1} = M_;
    end
    linelength = max(abs(M(1:3,4)))/10.0;
    lineWidth = 1;
    pos_list = [];
    for i =1:1:n
        exps = eye(4);
        for j =1:1:i
            exps = exps*exp_list{j};
        end
        Ti = T0*exps*Mlist{i};
        
        pos_list=[pos_list; Ti(1:3,4)'];
    end
    Teef = FKinSpace(M,Slist,thetalist);
    pos_eef = T0*Teef(1:4,4);
    pos_list=[pos_list; pos_eef(1:3)'];
    drawAxis(Teef,linelength ,lineWidth*2,T0,1,txt,ax);
    plot3(ax,pos_list(:,1),pos_list(:,2),pos_list(:,3),"k:",LineWidth = lineWidth*2)
    isBase = 0;
    for i =1:1:n
        exps = eye(4);
        for j =1:1:i
            exps = exps*exp_list{j};
        end
        Ti = exps*Mlist{i};
        if isBase ==0
            drawAxis(Ti,linelength/2, lineWidth*2,T0,0.3,base_txt,ax);
            isBase = 1;
        else
            drawAxis(Ti,linelength/2, lineWidth*2,T0,0.3,"",ax);
        end

    end    
end

