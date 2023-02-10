function Hc=PassiveConstarintJacobian(L,theta)
    H1T = [-L{1}*sin(theta{1}) ; L{1}*cos(theta{1})];
    H2T = [-L{2}*sin(theta{2}) ; L{2}*cos(theta{2})];
    O = zeros(size([1;H1T]' ))
    Hc = [[1;H1T],[0;0;0];[0;0;0],[1;H2T] ]
end