function drawRobot(M,Slist,pos_list,q)
      prevTi=eye(4);
      plot(0,0,'ks');hold on;
      plot(0,0,'k*');hold on;
      for i = 2:1:length(pos_list)
          Mi = eye(4);
          Mi(1:3,4) = pos_list(i,:)';
          Ti = FKinSpace(Mi,Slist(:,1:i),q(1:i));

          plot(Ti(1,4),Ti(2,4),'ko');
          plot([prevTi(1,4),Ti(1,4)] ,[prevTi(2,4),Ti(2,4)],'k-');
          prevTi = Ti;
      end
      Ti = FKinSpace(M,Slist,q);
      plot(Ti(1,4),Ti(2,4),'ko');
      plot([prevTi(1,4),Ti(1,4)] ,[prevTi(2,4),Ti(2,4)],'k-');
end