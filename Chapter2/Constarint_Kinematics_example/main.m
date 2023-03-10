%% Modern Robotics Setup
LL = 1.0 % Left Link Length
RL = 1.0 % Right Link Length
Left_w = [0 0 1; 0 0 1 ; 0 0 1; 0 0 1; 0 0 1; 0 0 1]
Right_w = [0 0 1; 0 0 1 ; 0 0 1; 0 0 1; 0 0 1; 0 0 1]
Left_p = [0 0 0 ;LL 0 0;2*LL 0 0;3*LL 0 0;4*LL 0 0;5*LL 0 0]
Right_p = [0 0 0 ;-RL 0 0;-2*RL 0 0;-3*RL 0 0;-4*RL 0 0;-5*RL 0 0]
Left_Slist = []
Right_Slist = []
for i = 1:1:length(Left_w)
    v = -cross(Left_w(i,:)',Left_p(i,:)') ;
    Left_Slist = [Left_Slist,[Left_w(i,:)'; v]]
    v = -cross(Right_w(i,:)',Right_p(i,:)') ;
    Right_Slist = [Right_Slist,[Right_w(i,:)'; v]]
end
Left_M = [1 0 0 6*LL; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
Right_M = [1 0 0 -6*RL; 0 1 0 0 ; 0 0 1 0; 0 0 0 1];
Left_Blist = Adjoint(TransInv(Left_M))*Left_Slist;
Right_Blist = Adjoint(TransInv(Right_M))*Right_Slist;

%% {0} to base Configuration {L}, {R}
T0L = [1 0 0 -3; 0 1 0 0 ;0 0 1 0 ;0 0 0 1]
T0R = [1 0 0 +3; 0 1 0 0 ;0 0 1 0 ;0 0 0 1]
%% {0} to box Configuration {b} 
T0b = RpToTrans(eul2rotm([0,0,0]),[0,3,0]');%[1 0 0 0; 0 1 0 3 ;0 0 1 0 ;0 0 0 1];
Tb0 = TransInv(T0b);
%%  box Configuration {b} to {Leef}, {Reef} 
TbLeef = [1 0 0 -1; 0 1 0 0 ;0 0 1 0 ;0 0 0 1];
TbReef = [1 0 0 1; 0 1 0 0 ;0 0 1 0 ;0 0 0 1];
TLLeef = TransInv(T0L)*T0b*TbLeef
TRReef = TransInv(T0R)*T0b*TbReef

%% Robot Initial Setup
q_l = [pi/2,-pi/6,-pi/6,-pi/6,-pi/6,-pi/6]';
q_r = [-pi/2,pi/6,pi/6,pi/6,pi/6,pi/6]';
%% Robot Inverse Kinematics
q_l=wrapToPi(IKinSpace(Left_Slist,Left_M,TLLeef,q_l,0.01,0.001));
q_r=wrapToPi(IKinSpace(Right_Slist,Right_M,TRReef,q_r,0.01,0.001));
%% Robot Forward Kinematics
T_L = FKinSpace(Left_M,Left_Slist,q_l)
T_R = FKinSpace(Right_M,Right_Slist,q_r)
%% Robot Jacobian 
Jb_L = JacobianBody(Left_Blist,q_l);
Jb_R = JacobianBody(Right_Blist,q_r);

%% Simulation Setup
dt = 0.001
fignum = 1;
f=figure(fignum);
f.Position = [680 603 793 375]
ax = axes();
plot(0,0)
hold(ax)
viewAngle = [0 ,90]
axisLimit = [-7 7 0 7 -7  7 ]
daspect([1,1,1])
view(viewAngle)
axis(axisLimit)
cla
t = 0;
step_count = 0;
for i = 1:1:1000
    t = t+dt;

    Vb_b = [0 0 exp(-5*t) 0 0 0]'; % Box Body Twist

    Vb_Leefb = Adjoint(TransInv(TbLeef))*Vb_b;
    Vb_Reefb = Adjoint(TransInv(TbReef))*Vb_b;
    qdot_l = pinv(Jb_L)*(Vb_Leefb);
    Ha= Adjoint(TbLeef)*Jb_L; % Actuated Jacobian Matrix, q_a = q_l
    Hp= -Adjoint(TbReef)*Jb_R; % Passive Jacobian Matrix, q_p = q_r
    qdot_r= -pinv(Hp)*Ha*qdot_l;
    
    [q_l,qdot_l]=EulerStep(q_l,qdot_l,[0,0,0,0,0,0]',dt);
    [q_r,qdot_r]=EulerStep(q_r,qdot_r,[0,0,0,0,0,0]',dt);
    T_L = FKinSpace(Left_M,Left_Slist,q_l);
    T_R = FKinSpace(Right_M,Right_Slist,q_r);
    Jb_L = JacobianBody(Left_Blist,q_l);
    Jb_R = JacobianBody(Right_Blist,q_r);    
    
    %% DRAW
    if step_count > 40
    cla
    drawRobot(Left_M,Left_Slist,q_l,T0L,Left_p,"\{Leef\}","\{L\}",ax,[1,1,1,1,1,1])
    drawRobot(Right_M,Right_Slist,q_r,T0R,Right_p,"\{Reef\}","\{R\}",ax,[2,2,2,2,2,2])
    width = 2;
    height = 2;
    drawBox(T0L*T_L*TransInv(TbLeef),width,height,ax)
    text(-2,6,0,"Vb_b = ["+string(Vb_b(1))+","+string(Vb_b(2))+","+string(Vb_b(3))+","+string(Vb_b(4))+","+string(Vb_b(5))+","+string(Vb_b(6))+"]")
    text(-2,6-0.5,0,"Vb_{Leef} = ["+string(Vb_Leefb(1))+","+string(Vb_Leefb(2))+","+string(Vb_Leefb(3))+","+string(Vb_Leefb(4))+","+string(Vb_Leefb(5))+","+string(Vb_Leefb(6))+"]")
    hold(ax,"on")
    drawnow
    step_count = 0;
    end
    step_count = step_count+1;
end
