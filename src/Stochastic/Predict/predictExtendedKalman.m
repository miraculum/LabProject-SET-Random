    
%PREDICTEXTENDEDKALMAN First order Extended Kalman predict
%
% Syntax:
%   [x,P] = PREDICTEXTENDEDKALMAN(x,P,u,f,Q,w)
%
% In:
%   x - mean state estimate of previous step
%   P - state covariance of previous step
%   u - control vector
%   f - xean prediction E[f(x[k-1],q=0)] function handle. E.g. f = @(x,u) x + log(x) + u
%   Q - Process noise covariance matrix              (optional, default zero)
%   w - process noise                                (optional, default zero)
%   f1- first derivative of f in the form of function handle.        (calculated on the first run)

%   w - Process noise                                (optional, default zero)
%
% Out:
%   x - Updated state mean
%   P - Updated state covariance
%   
% Description:
%   Perform first order Extended Kalman Filter prediction step.
%
% See also:
% updateExtendedKalmam



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
        %F- nxm
        %P- mxm
        %Q- nxn

        if size(F,2) ~= size(P,1) && size(F,2) ~= size(P,2) && size(Q,1) ~= size(F,1) && size(Q,2) ~= size(F,1)
            errorMessage("matrix size error in predictExtendedKalman");
        end

        P = F * P * F' + Q;

    else
        errorMessage('predictExtendedKalman requires at least 4 arguments');
    end
    