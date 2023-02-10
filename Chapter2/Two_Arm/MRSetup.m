function [M,Slist,Blist,Mlist,Glist,p_list]=MRSetup()
LL = 1;
LR = 1;
L1 = 2;
L2 = 1;
L3 = 0.5;
L_p_list = [-LL 0 0;-LL L1 0;-LL+L2 L1 0] ;
L_w_list = [0 0 1;0 0 1; 0 0 1];
L_Slist = w_p_to_Slist(L_w_list,L_p_list);
R_p_list = [LR 0 0;LR L1 0;LR-L2 L1 0] 
R_w_list = [0 0 1;0 0 1; 0 0 1];
R_Slist = w_p_to_Slist(R_w_list,R_p_list);
L_M = [0 1 0 -LL+L2+L3; 
       -1 0 0 L1;
       0 0 1 0;
       0 0 0 1]
R_M = [0 -1 0  LR-L2-L3; 
       1 0 0 L1;
       0 0 1 0;
       0 0 0 1]
L_M01 = [1 0 0 -LL;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1] ;

L_M02 = [1 0 0 -LL;
         0 1 0 L1;
         0 0 1 0;
         0 0 0 1] ;
L_M12 = TransInv(L_M01)*L_M02;
L_M03 = [1 0 0 -LL+L2;
         0 1 0 L1;
         0 0 1 0;
         0 0 0 1] ;
L_M23 = TransInv(L_M02)*L_M03;
L_M04 = [1 0 0 -LL+L2+L3;
         0 1 0 L1;
         0 0 1 0;
         0 0 0 1] ;
L_M34 = TransInv(L_M03)*L_M04;

R_M01 = [1 0 0 LR;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1] ;

R_M02 = [1 0 0 LR;
         0 1 0 L1;
         0 0 1 0;
         0 0 0 1] ;
R_M12 = TransInv(R_M01)*R_M02;
R_M03 = [1 0 0 LR-L2;
         0 1 0 L1;
         0 0 1 0;
         0 0 0 1] ;
R_M23 = TransInv(R_M02)*R_M03;
R_M04 = [1 0 0 LR-L2-L3;
         0 1 0 L1;
         0 0 1 0;
         0 0 0 1] ;
R_M34 = TransInv(R_M03)*R_M04;


L_Mlist = cat(3, L_M01, L_M12, L_M23, L_M34); 
R_Mlist = cat(3, R_M01, R_M12, R_M23, R_M34); 

L_G1 = diag([0.0001, 0.0001, 0.0001, 2,2,2]);
L_G2 = diag([0.0001, 0.0001, 0.0001, 1,1,1]);
L_G3 = diag([0.0001, 0.0001, 0.0001, 0.5,0.5,0.5]);
R_G1 = diag([0.0001, 0.0001, 0.0001, 2,2,2]);
R_G2 = diag([0.0001, 0.0001, 0.0001, 1,1,1]);
R_G3 = diag([0.0001, 0.0001, 0.0001, 0.5,0.5,0.5]);
L_Glist = cat(3, L_G1, L_G2, L_G3);
R_Glist = cat(3, R_G1, R_G2, R_G3);

Slist{1}=L_Slist;
Slist{2}=R_Slist;
Blist{1}=Adjoint(TransInv(L_M))*L_Slist;
Blist{2}=Adjoint(TransInv(R_M))*R_Slist;
Mlist{1}=L_Mlist;  
Mlist{2}=R_Mlist;  
Glist{1} = L_Glist;  
Glist{2} = R_Glist;  
M{1} = L_M;
M{2} = R_M;
p_list{1} = L_p_list;
p_list{2} = R_p_list;

end