
function [x, p, y] = KBPredictedX(F, xkmin1, P, H, Rk, u, Qk, B, z)

    [x, p, y] = KalmanBucy(F, xkmin1, P, H, Rk, u, Qk, B, z);
