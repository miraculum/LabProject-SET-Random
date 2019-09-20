    
                        % Extended Kalman predict %

% In:
%   x - mean state estimate of previous step
%   P - state covariance of previous step
%   u - control vector
%   f - mean prediction E[f(x[k-1],q=0)] function handle. 
%        E.g. f = @(x,u) x + log(x) + u
%   f1- first derivative of f in the form of function handle.
%   Q - Process noise covariance matrix            (optional, default zero)
%   w - process noise                              (optional, default zero)
%
% Out:
%   x - Updated state mean
%   P - Updated state covariance


function [x,P,f1] = predictExtendedKalman(x,P,u,f,Q,f1)

    % Number of inputs control 
    if nargin >= 4
        
        if nargin < 5
            Q = [];
        end

        if isempty(u)
          errorMessage("Control vetor 'u' is empty");
        end
        
        
        x = f(x,u);
        F = f1(x,u);
    

        %Matrix size test

        if size(F,2) ~= size(P,1) && size(F,2) ~= size(P,2) && size(Q,1)...
                ~= size(F,1) && size(Q,2) ~= size(F,1)
            errorMessage("matrix size error in predictExtendedKalman");
        end

        P = F * P * F' + Q;

    else
        errorMessage('predictExtendedKalman requires at least 4 arguments');
    end
    