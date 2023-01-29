clear
%% calc FK

Slist{1} = [0 0 1 1 0 0;
            0 0 0 0 -1 0;
            0 0 1 0.5 0 0]';

Slist{2} = [0 0 1 -1/2  sqrt(3)/2   0;
            0 0 0  sqrt(3)/2   1/2   0;
            0 0 1 -1/4  sqrt(3)/4   0]';
Slist{3} = [0 0 1 -1/2   -sqrt(3)/2   0;
            0 0 0  -sqrt(3)/2  1/2    0
            0 0 1 -1/4   -sqrt(3)/4   0]';
q{1} = [pi/4,-0.0,-pi/2]';
q{2} = [pi/4,-0.0,-pi/2]';
q{3} = [pi/4,-0.0,-pi/2]';
p_list{1} = [0 1 0; 0 0.5 0; 0 0.5 0]
p_list{2} = [-sqrt(3)/2 -0.5 0; -sqrt(3)/4 -0.25 0; -sqrt(3)/4 -0.25 0]
p_list{3} = [sqrt(3)/2 -0.5 0;   sqrt(3)/4 -0.25 0;   sqrt(3)/4 -0.25 0]
M{1} = [1 0 0 0; 
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M{2} = [1 0 0 0; 
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
M{3} = [1 0 0 0; 
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
Blist{1} = Adjoint(TransInv(M{1}))*Slist{1};
Blist{2} = Adjoint(TransInv(M{2}))*Slist{2};
Blist{3} = Adjoint(TransInv(M{3}))*Slist{3};
TargetT = RpToTrans(eul2rotm([pi/6,0,0]),[0,0,0]')
q0=IKinSpace(Slist{1},M{1},TargetT,[pi/4,-0.0,-pi/2]',0.01,0.001);
q{1} = q0;
q{2} = q0;
q{3} = q0;

dt = 0.001
endTime = 5
drawCount = 0
figure(1)
ax = axes();
hold(ax)
for t = linspace(0,endTime,floor(endTime/dt))
    
    q_a = get_q_a(q);
    q_p = get_q_p(q);
    Jb{1} = JacobianBody(Blist{1},q{1});
    Jb{2} = JacobianBody(Blist{2},q{2});
    Jb{3} = JacobianBody(Blist{3},q{3});
    O = zeros(size(Jb{1}));
    actuated_num = [2,5,8];
    passive_num = [1,3,4,6,7,9];
    Jc = [Jb{1} -Jb{2}    O;
          O     -Jb{2} Jb{3}]
    Ha = Jc(:,actuated_num);
    Hp = Jc(:,passive_num);
    g = -pinv(Hp)*Ha;
    e1 = [0 1 0]';
    Ja = Jb{1}*[g(1,:); e1';g(2,:)]
    VT = [0 0 -0.1 0 0 0]';
    qdot_a = pinv(Ja)*VT;
    qdot_p = g*qdot_a;
    [q_a,qdot_a] = EulerStep(q_a,qdot_a,zeros(size(q_a)),dt);
    [q_p,qdot_p] = EulerStep(q_p,qdot_p,zeros(size(q_p)),dt);
    q = get_q(q_a,q_p);
    if drawCount > 50
        cla
        drawRobot(ax,M,Slist,q,p_list);
        daspect([1,1,1])
        drawnow;
        grid on;
        drawCount = 0
    end
    drawCount = drawCount+1
end

%% draw
% figure(1)
% ax = axes();
% hold(ax)
% drawRobot(ax,M,Slist,q,p_list);
% daspect([1,1,1])
% grid on;

function q_a = get_q_a(q)
    q = reshape(cell2mat(q),9,1)
    S_a = [0 1 0 0 0 0 0 0 0;
           0 0 0 0 1 0 0 0 0
           0 0 0 0 0 0 0 1 0];
    q_a = S_a*q;
end
function q_p = get_q_p(q)
    q = reshape(cell2mat(q),9,1)
    S_p = [1 0 0 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0 0;
           0 0 0 1 0 0 0 0 0;
           0 0 0 0 0 1 0 0 0;
           0 0 0 0 0 0 1 0 0;
           0 0 0 0 0 0 0 0 1];
    q_p = S_p*q;
end
function q = get_q(q_a,q_p)
    
    S = [0 0 0 1 0 0 0 0 0;
           1 0 0 0 0 0 0 0 0;
           0 0 0 0 1 0 0 0 0;
           0 0 0 0 0 1 0 0 0;
           0 1 0 0 0 0 0 0 0;
           0 0 0 0 0 0 1 0 0;
           0 0 0 0 0 0 0 1 0;
           0 0 1 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 1];
    q_ = S*[q_a;q_p];
    q={};
    q{1} =  q_(1:3);
    q{2} =  q_(1+3:3+3);
    q{3} =  q_(1+3+3:3+3+3);
end
