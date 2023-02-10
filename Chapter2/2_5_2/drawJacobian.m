function drawJacobian(M,Slist,q,J,J2)
    T = FKinSpace(M,Slist,q);
%     [qdot1,qdot2,qdot3] = sphere(100);
%     qdot1 = reshape(qdot1,[],1)
%     qdot2 = reshape(qdot2,[],1)
%     qdot3 = reshape(qdot3,[],1)
%     qdot = [qdot1';qdot2';qdot3'];
%     drawEllipse(J(1:2,1:2),'r',T(1,4),T(2,4))
    theta = 0:0.001:2*pi
    qdot1 = cos(theta);
    qdot2 = sin(theta);
    qdot3 = zeros(size(qdot1));
    qdot = [qdot1;qdot2;qdot3]
    V = J*qdot
    F = pinv(J')*qdot
    fill(V(1,:)+ T(1,4),V(2,:)+ T(2,4),'b--',FaceAlpha=0.1);
%     fill(F(1,:)+ T(1,4),F(2,:)+ T(2,4),'r--',FaceAlpha=0.1);
%     V = J2*qdot(1:2,:)
%     fill(V(1,:)+ T(1,4),V(2,:)+ T(2,4),'b:',FaceAlpha=0.1);
    F = pinv(J2')*qdot(1:2,:)
%     fill(F(1,:)+ T(1,4),F(2,:)+ T(2,4),'r:',FaceAlpha=0.1);

    formatSpec = "---------------Jacobian-----------------\n %f \t %f \t %f \n %f \t %f \t %f ";
    str = sprintf(formatSpec,J(1,1),J(1,2),J(1,3),J(2,1),J(2,2),J(2,3));
    text(3,-3.5,str)
%     plot(qdot(1,:)+ T(1,4),qdot(2,:)+ T(2,4));
end

function drawEllipse(J,color,x,y)
    [U,SIGMA,V]=eig(J);
    u1 = U(:,1);
    u2 = U(:,2);
    sigma1 = SIGMA(1,1);
    sigma2 = SIGMA(2,2)
    quiver([x],[y],[u1(1)*sigma1],[u1(2)*sigma1],0,color,'LineWidth',2,"MaxHeadSize",500)
    quiver([x],[y],[u2(1)*sigma2],[u2(2)*sigma2],0,color,'LineWidth',2,"MaxHeadSize",500)
    [X,Y] = calculateEllipse(x, y, sigma2, sigma1, acos(u1'*u2)*180/pi,100)
    plot(X,Y,color)

end

function [X,Y] = calculateEllipse(x, y, a, b, angle, steps)
    %# This functions returns points to draw an ellipse
    %#
    %#  @param x     X coordinate
    %#  @param y     Y coordinate
    %#  @param a     Semimajor axis
    %#  @param b     Semiminor axis
    %#  @param angle Angle of the ellipse (in degrees)
    %#

    narginchk(5, 6);
    if nargin<6, steps = 36; end

    beta = -angle * (pi / 180);
    sinbeta = sin(beta);
    cosbeta = cos(beta);

    alpha = linspace(0, 360, steps)' .* (pi / 180);
    sinalpha = sin(alpha);
    cosalpha = cos(alpha);

    X = x + (a * cosalpha * cosbeta - b * sinalpha * sinbeta);
    Y = y + (a * cosalpha * sinbeta + b * sinalpha * cosbeta);

    if nargout==1, X = [X Y]; end
end