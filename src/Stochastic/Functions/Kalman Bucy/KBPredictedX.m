
function [x, p, y] = KBPredictedX(P, F, xkmin1, B, u, Qk, H, Rk, z)

[predx, predp] = predictKalman(P, xkmin1, F, B, u, Qk);
[x, p, y] = updateKalman(predx, predp, H, Rk, z);