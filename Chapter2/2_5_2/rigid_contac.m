J = [0      0.7071  0   0 ; 
    2.4142  1.7071  1   0;
    0       0       0   -0.25 ; 
    0       0       0   0.25]

A  =  [eye(2);eye(2)];
C1 = pinv(A)*J

% C1 =
% 
%          0    0.3535         0   -0.1250
%     1.2071    0.8535    0.5000    0.1250
rank(J)
% ans =
% 
%      3
C2 = null(A')'*J
% C2 =
% 
%          0   -0.5000         0   -0.1768
%    -1.7071   -1.2071   -0.7071    0.1768
sqrtOmega = eye(2);
C1C2sqrtOmega = C1*null(C2)*sqrtOmega;
% C1C2sqrtOmega =
% 
%    -0.0561   -0.4470
%     0.0561    0.4470
%% SVD
[U,SIGMA,V] = eig(C1C2sqrtOmega);
