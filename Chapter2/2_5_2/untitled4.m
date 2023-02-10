syms px py 

Slist = [ 0 0 0 1 0 0 ;
          0 0 1 py -px 0]';
Blist = Adjoint(TransInv(M))*Slist
M = eye(4);