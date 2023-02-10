function [leg_p,connection_list,M,Slist,Blist]=MRSetup()
p_list = [sqrt(3)/2 -0.5 0;
         0 1 0;
         -sqrt(3)/2 -0.5 0;
         sqrt(3)/2 0.5 1;
         -sqrt(3)/2 0.5 1;
         0 -1 1];

connection_list = [1,4;
                   2,4;
                   2,5;
                   3,5;
                   3,6;
                   1,6]

LEG_NUM = 6
leg_w_list = [1,0,0;
          0,1,0;
          0,0,1;
          0,0,0;
          1,0,0;
          0,1,0;
          0,0,1]
Slist={}
Blist={}
M={}
leg_p = {}
e_list=[]
for i =1:1:LEG_NUM
    ind=connection_list(i,:);
    leg_p_list = [p_list(ind(1),:);
                  p_list(ind(1),:);
                  p_list(ind(1),:);
                  0,0,0;
                  p_list(ind(2),:);
                  p_list(ind(2),:);
                  p_list(ind(2),:)];
    leg_p{i} = leg_p_list;
    ei = p_list(ind(2),:)-p_list(ind(1),:);
    ei = ei/norm(ei);
    e_list = [e_list;ei];
    leg_Slist = []
    for j = 1:1:length(leg_w_list)
        if j==4
            S = [0 0 0 ei]';
        else
            S = [leg_w_list(j,:)'; -cross(leg_w_list(j,:)',leg_p_list(j,:)')];
        end
        leg_Slist = [leg_Slist,S];
    end
    Slist{i} = leg_Slist;
    M_ = [-1 0 0 0;
         0 -1 0 0;
         0 0 1 1;
         0 0 0 1]
    Blist{i} = Adjoint(TransInv(M_))*leg_Slist;
    M{i} = M_;
end
e_list
end