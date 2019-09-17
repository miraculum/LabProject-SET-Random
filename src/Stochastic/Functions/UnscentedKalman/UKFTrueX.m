
function [xk, z] = UKFTrueX(f, h, xkmin1, u, w, v)

    xk = f(xkmin1, u) + w;
    z = h(xk) + v;