
function [x, p, f, y ] = UKFPredictedX(xkmin1, p, f, Qk, Rk, f_param, alpha, beta, kappa, mat, y)
    %Call predict and updte functions
    [x, p] = predictUnscentedKalman(xkmin1, p, f, Qk, f_param, alpha, beta, kappa, mat);
    [x, p] = updateUnscentedKalman(x, p, y, f, Rk, f_param, alpha, beta, kappa, mat);
    

