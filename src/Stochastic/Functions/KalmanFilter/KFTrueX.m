
function [xk, z] = KFTrueX(F, xkmin1, B, u, w, H, v)

xk = F * xkmin1 + B * u + w;
z = H * xk + v;