function [est, err, uncert] = predict(fun, struct)
% PROPAGATE applies estimator to parameters.
%
%   C = PROPAGATE(fun, parameteres) takes cell array of estimator function pointers and vector of parameters
 
%Preallocating memory for array
functionNumber = size(fun);
est = zeros(functionNumber);
err = zeros(functionNumber);
uncert = zeros(functionNumber); 
 
    for n = 1: length(fun)
        [est(n), err(n), uncert(n)] = fun{n}(struct(n)); 
    end
