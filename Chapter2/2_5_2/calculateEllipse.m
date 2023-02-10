function [x,y] = calculateEllipse(a, b,cx,cy,alpha)
    theta = 0:0.01:2*pi;
    x = a*cos(theta);
    y = b*sin(theta);
    R = eul2rotm([alpha,0,0]);
    p = R*[x;y;zeros(size(x))];
    x = p(1,:)+cx;
    y = p(2,:)+cy;
    plot(x,y)
end