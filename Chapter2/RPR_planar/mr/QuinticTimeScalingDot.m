function sdot = QuinticTimeScalingDot(Tf, t)
sdot = 10 *3/Tf* (t / Tf) ^ 2 - 15 * 4/Tf*(t / Tf) ^ 3 + 6 / Tf* 5*(t / Tf) ^ 4;
end