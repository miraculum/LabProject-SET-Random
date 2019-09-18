                      % Kalman Filter predict %
                      
% In:
%   x - Nx1 mean state estimate of previous step
%   P - NxN state covariance of previous step
%   F - Transition matrix of discrete model (optional, default identity)
%   B - Input effect matrix                 (optional, default identity)
%   u - Constant input                      (optional, default empty)
%   Q - Process noise of discrete model     (optional, default zero)

% Out:
%   X - Predicted state mean
%   P - Predicted state covariance

function [x, P] = predictKalman(P, x, F, B, u, Q)

  % Number of inputs control 
  if nargin < 3
    F = [];
  end
  if nargin < 4
    B = [];
  end
  if nargin < 5
    u = [];
  end
  if nargin < 6
    Q = [];
  end

  % Apply defaults
    if isempty(F)
        F = eye(size(x,1));
    end
    
    if isempty(B)
        B = eye(size(x,1),size(u,1));
    end
    
    if isempty(Q)
        Q = eye(size(F',1));
    end

    %predicted state estimate
    x = F * x + B * u;
         
    %predicted error covariace
    P = F * P * F' + Q;
    

    
