                      % Kalman Bucy filter %
                      
% In:
%   F - Transition matrix of discrete model (optional, default identity)
%   X - Nx1 mean state estimate
%   P - NxN state covariance of previous step
%   H - Measurement matrix.
%   R - Measurement noise covariance.
%   u - Constant input                      (optional, default empty)
%   B - Input effect matrix                 (optional, default identity)
%   Q - Process noise of discrete model     (optional, default zero)
%   z - observation of the true state x

% Out:
%   X  - Updated state mean
%   P  - Updated state covariance
%   Y  - Measurement post-fit residual

function [X, P, y] = KalmanBucy(F, X, P, H, R, u, Q, B, z)

    K = P * H' / R;
    
    X = F * X + B * u + K * (z - H * X);
    P = F * P + P * F' + Q - K * R * K';
    y = z - H * X;