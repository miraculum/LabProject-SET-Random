
function [xk] = UKFTrueX(f, xkmin1, u, w)

    xk = f(xkmin1, u) + w;
    