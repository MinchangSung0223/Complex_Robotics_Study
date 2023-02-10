function drawBox(T,width,height,ax)

    linelength = max(abs(T(1:3,4)))/5;
    linewidth = max(abs(T(1:3,4)))/2;
    drawAxis(T,linelength,linewidth,eye(4),1,"\{b\}",ax,0);
    p1 = T*[width/2,-height/2,0,1]';
    p2 = T*[width/2,height/2,0,1]';
    p3 = T*[-width/2,height/2,0,1]';
    p4 = T*[-width/2,-height/2,0,1]';
    p_list = [p1';p2';p3';p4';p1'];
    p_list(:,4) = [];

    fill3(ax,p_list(:,1),p_list(:,2),p_list(:,3),'k',FaceAlpha=0.1);

end
