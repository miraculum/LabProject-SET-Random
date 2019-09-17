%PREDICTEXTENDEDKALMAN First order Extended Kalman update
%
% Syntax:
%   [K,x,P,H1] = UPDATEEXTENDEDKALMAN(x,P,u,h,R,v,h1)
%
% In:
%   x - mean state estimate of previous step
%   P - state covariance of previous step
%   u - control vector
%   h - valid function handle
%   R - Observation noise covariance matrix           (optional, default zero)
%   v - Observation noise
%   h1- first derivative of h in the form of function handle.        (calculated the first time the algorithm runs)


% Out:
%   x - Updated state mean
%   P - Updated state covariance
%   f1- first derivative of f in the form of function handle.        

% Description:
%   Perform first order Extended Kalman Filter update step.
%
% See also:
% predictExtendedKalmam



function [x,P,h1] = updateExtendedKalman(x,P,u,h,R,z,h1)

    % Number of inputs control     
    if nargin < 4
        errorMessage('updateExtendedKalman requires at least 4 arguments');
    end
    if nargin < 5
        R = [];
    end
    
    if isempty(u)
      errorMessage("Control vetor 'u' is empty");
    end

        H = h1(x);
            
        y = z - h(x); 
        S = H * P * H' + R;                 %Innovation (or residual) covariance
        K = P * H' / S;                     %Near-optimal Kalman gain
        x = x + K * y;                      %Updated state estimate		
        I = eye(size(K,1));
        P = (I - K * H) * P;                %Updated covariance estimate	
    end
    

