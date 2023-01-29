function sddot = QuinticTimeScalingDdot(Tf, t)
sddot = 10 *3/Tf*2/ Tf* (t / Tf)  - 15 * 4/Tf*3/ Tf*(t / Tf) ^ 2 + 6 / Tf* 5*4/ Tf*(t / Tf) ^ 3;
end