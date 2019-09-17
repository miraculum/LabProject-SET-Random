
function [x, p, f1, h1] = EFKPredictedX(P, f, xkmin1, u, Qk, h, Rk, z, f1, h1)
    [x, p, f1] = predictExtendedKalman(xkmin1, P, u, f, Qk, f1);
    [x, p, h1] = updateExtendedKalman(x, p, u, h, Rk, z, h1);