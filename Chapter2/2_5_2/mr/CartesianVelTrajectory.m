function traj = CartesianVelTrajectory(Xstart, Xend, Tf, N, method)

timegap = Tf / (N - 1);
traj = cell(1, N);
[Rstart, pstart] = TransToRp(Xstart);
[Rend, pend] = TransToRp(Xend);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf,timegap * (i - 1));
        sdot = CubicTimeScalingDot(Tf,timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf,timegap * (i - 1));
        sdot = QuinticTimeScalingDot(Tf,timegap * (i - 1));
    end
    Rs = Rstart * MatrixExp3(MatrixLog3(Rstart'*Rend) * s);
    Rdot = MatrixLog3(Rstart'*Rend)*Rs;
%     w = so3ToVec(Rdot*Rs'*sdot); %% [w] = dotR * R^T
    w = so3ToVec(MatrixLog3(Rstart'*Rend)*sdot); %% [w] = dotR * R^T

    traj{i} ...
    = [w ; ...
       sdot * (pend - pstart)];
end
end