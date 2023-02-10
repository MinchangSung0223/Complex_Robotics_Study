clear
addpath("mr")
[M,Slist,Blist,Mlist,Glist,p_list]=MRSetup();
q{1} = [pi/3,0,-pi/3]';
q{2} = [-pi/3,0,pi/3]';
T1 = [0 1 0 -1;
      -1 0 0 2;
      0 0 1 0;
      0 0 0 1]
T2 = [0 -1 0 1;
      1 0 0 2;
      0 0 1 0;
      0 0 0 1]
% T1= FKinSpace(M{1},Slist{1},q{1})
% T2= FKinSpace(M{2},Slist{2},q{2})
[q{1},ret1]=IKinSpace(Slist{1},M{1},T1,q{1},0.01,0.001);
[q{2},ret2]=IKinSpace(Slist{2},M{2},T2,q{2},0.01,0.001);

%% Draw
fig = figure(1);
ax = axes();
hold(ax);
drawRobot(ax,M,Slist,p_list,q);
daspect([1,1,1])
axis([-5 5 -0.5 5])
grid on;

%% Forward Dynamics
endTime = 5;
dt = 0.001;
qdot{1} =zeros(3,1);
qdot{2} =zeros(3,1);
qddot{1} =zeros(3,1);
qddot{2} =zeros(3,1);
tau{1} =zeros(3,1);
tau{2} =zeros(3,1);
tau_grav{1}=zeros(3,1);
tau_grav{2}=zeros(3,1);
g = [0 -1 0]';
drawCount= 50;
count = 0;
Ftip{1} = [0 0 0 0 0.001 0]';
Ftip{2} = [0 0 0 0 0.001 0]';
vtip{1} = zeros(6,1);
vtip{2} = zeros(6,1);
Fc{1} = zeros(6,1);
Fc{2} = zeros(6,1);
FK{1} = T1;
FK{2} = T2;
vT{1} = [0,0,0,0,0,0]';
vT{2} = [0,0,0,0,0,0]';
T_T{1} = [0 1 0 -1;
        -1 0 0 0;
        0 0  1 0;
        0 0 0 1]
T_T{2} = [-1 0 0 1;
        0 1 0 0;
        0 0  1 0;
        0 0 0 1]
Ftip_now{1} = zeros(6,1);
Ftip_now{2} = zeros(6,1);
fT{1} = zeros(6,1);
fT{2} = zeros(6,1);
Mass{1} = eye(3);
Mass{2} = eye(3);
invMass{1} = eye(3);
invMass{2} = eye(3);
for t = linspace(0,endTime , floor(endTime/dt))
    
    for i =1:1:2
     Jb{i} = JacobianBody(Blist{i},q{i});
     vtip{i} = Jb{i} *qdot{i};
     Ftip_now{i} = pinv(Jb{i}') *tau{i};
     vT{i} = Adjoint((T_T{i}))* Jb{i}*qdot{i}; 
     fT{i} = Adjoint((T_T{i}))* Ftip_now{i}; 
     Mass{i} = MassMatrix(q{i}, Mlist{i}, Glist{i}, Slist{i});
     invMass{i} = inv(Mass{i});
    end
    Jc = [Adjoint((T_T{1}))* Jb{1} -Adjoint((T_T{2}))* Jb{2}];
    A = Jc;
    Mass_ = [Mass{1} zeros(3,3);zeros(3,3) Mass{2}];
    invMass_ = inv(Mass_);
    tau_ = [tau{1}; tau{2}];
    qddot_ = [qddot{1}; qddot{2}];
    lambda =pinv(A*invMass_*A')*(A*invMass_*(tau_)-A*qddot_);
    F_c = A'*lambda;
    for i =1:1:2
     
     FK{i} = FKinSpace(M{i},Slist{i},q{i});
%     disp(vtip{1})
     tau_grav{i}  = InverseDynamics(q{i}, qdot{i}, qddot{i}, ...
                                   g, [0,0,0,0,0,0]', Mlist{i}, Glist{i}, Slist{i});

     
     FK1 = FK{1};

     FK2 = FK{2};

     tau{i} = tau_grav{i}+Jb{i}'*Ftip{i};     
     qddot{i} = ForwardDynamics(q{i}, qdot{i}, tau{i}, ...
                                       g, [0,0,0,0,0,0]', Mlist{i}, Glist{i}, Slist{i});
     [q{i},qdot{i}]=EulerStep(q{i},qdot{i},qddot{i},dt);
    end
    if count >drawCount
        cla
        drawRobot(ax,M,Slist,p_list,q);
        drawBox([1 0 0 0;0 1 0 2; 0 0 1 0; 0 0 0 1],2,1,ax);
        drawnow;
        count = 0;
    end
    count = count+1;
end
