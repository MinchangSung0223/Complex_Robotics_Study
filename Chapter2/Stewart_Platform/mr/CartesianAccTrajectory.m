function traj = CartesianAccTrajectory(Xstart, Xend, Tf, N, method)

timegap = Tf / (N - 1);
traj = cell(1, N);
[Rstart, pstart] = TransToRp(Xstart);
[Rend, pend] = TransToRp(Xend);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf,timegap * (i - 1));
        sdot = CubicTimeScalingDot(Tf,timegap * (i - 1));
        sddot= CubicTimeScalingDdot(Tf,timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf,timegap * (i - 1));
        sdot = QuinticTimeScalingDot(Tf,timegap * (i - 1));
        sddot = QuinticTimeScalingDdot(Tf,timegap * (i - 1));
    end
    w0T = MatrixLog3(Rstart'*Rend);
    Rs = Rstart * MatrixExp3(w0T * s);
    dRds = w0T*Rs;
    dRds_T = Rs'*w0T';
    ddRdsds = w0T*w0T*Rs;
    dw = ddRdsds*Rs'+dRds*dRds_T;

      w = so3ToVec(dRds*Rs'*sddot+dw*sdot*sdot); %% [w] = dotR * R^T
      
%       w = so3ToVec(Rstart*w0T*Rstart'*sddot+Rstart*(w0T*w0T+w0T*w0T')*Rstart'*sdot*sdot)
%     w = so3ToVec(MatrixLog3(Rstart'*Rend)*sddot+MatrixLog3(Rstart'*Rend)*sdot*sdot)
    traj{i} ...
    = [w ; ...
       sddot * (pend - pstart)];
end
end