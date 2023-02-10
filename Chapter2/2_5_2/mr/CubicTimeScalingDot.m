function sdot = CubicTimeScalingDot(Tf, t)
sdot = 3*2 / Tf * (t / Tf)  - 2 *3/ Tf* (t / Tf) ^ 2;
end