function drawAxis(T,linelength ,lineWidth,T0,opacity,txt,ax)
    p0 = T0*T*[0,0,0,1]';
    px = T0*T*[linelength,0,0,1]';
    py = T0*T*[0,linelength,0,1]';
    pz = T0*T*[0,0,linelength,1]';
    plot3(ax,[p0(1) px(1)],[p0(2) px(2)],[p0(3) px(3)],Color=[1,0,0,opacity],LineWidth=lineWidth); 
    plot3(ax,[p0(1) py(1)],[p0(2) py(2)],[p0(3) py(3)],Color=[0,1,0,opacity],LineWidth=lineWidth);
    plot3(ax,[p0(1) pz(1)],[p0(2) pz(2)],[p0(3) pz(3)],Color=[0,0,1,opacity],LineWidth=lineWidth);
    
    plot3(ax,p0(1),p0(2),p0(3),'k.',MarkerSize=lineWidth*7);
    plot3(ax,p0(1),p0(2),p0(3),'y.',MarkerSize=lineWidth*3);
    text(p0(1)-linelength/5.0,p0(2)-linelength/5.0,p0(3)-linelength/5.0,txt,FontSize=12);
end