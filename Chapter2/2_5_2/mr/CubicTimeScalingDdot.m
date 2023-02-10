function sdot = CubicTimeScalingDdot(Tf, t)
sdot = 3*2 / Tf * (1.0 / Tf)  - 2 *3/ Tf*2/ Tf* (t / Tf) ;
end