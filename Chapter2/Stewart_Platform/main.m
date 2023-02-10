addpath('mr')
[p_list,connection_list,M,Slist,Blist]=MRSetup();
LEG_NUM = 6
dt = 0.001
initialR = [-1 0 0; 0 -1 0; 0 0 1]
TargetT = RpToTrans(eul2rotm([0 0 0])*initialR,[0 0 1]')
for i =1:1:LEG_NUM
  q{i} = zeros(7,1);
  q{i}=IKinSpace(Slist{i},M{i},TargetT,q{1},0.01,0.001)
end
endTime= 1
drawCount = 50
count = 0;
VT = [0 0.1 0.1 0 0 0]';
for t=linspace(0,endTime,floor(endTime/dt))
    [S_a,q_a] = get_qa(q);
    [S_p,q_p] = get_qp(q);
    Jb={};
    for i =1:1:LEG_NUM  
        Jb{i} = JacobianBody(Blist{i},q{i});
    end
    O = zeros(size(Jb{1}));
    Jc = [Jb{1}     -Jb{2}      O       O       O   O;
              O     -Jb{2}  Jb{3}       O       O   O;
              O          O  Jb{3}  -Jb{4}       O   O;
              O          O      O  -Jb{4}   Jb{5}   O;
              O          O      O       O   Jb{5}   -Jb{6}];
    Ha = Jc*S_a';
    Hp = Jc*S_p';
    q_ = reshape(cell2mat(q),42,1);
    g = -pinv(Hp)*Ha;
    e1 = [1 0 0 0 0 0]';
    Ja = Jb{1}*[ g(1,:) ;g(2,:) ;g(3,:) ;e1';g(4,:) ;g(5,:) ;g(6,:) ];
    VT = [0 0.1 0.1 0 0 0]';
    qdot_a = pinv(Ja)*VT;
    qdot_p = g*qdot_a;
    [q_a,qdot_a] = EulerStep(q_a,qdot_a,zeros(size(q_a)),dt);
    [q_p,qdot_p] = EulerStep(q_p,qdot_p,zeros(size(q_p)),dt);
    q= get_q(q_a,q_p,S_a,S_p);
    
    if count >drawCount
        %% DRAW
        figure(1);ax = axes();hold(ax);cla;
        drawRobot(ax,p_list,connection_list,M,Slist,q)
        grid on;view([40 28]);daspect([1,1,1]);
        drawnow;
        count=0;
    end
    count = count+1;
end
