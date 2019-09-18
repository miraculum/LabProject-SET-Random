                        % Unscented Kalman update %

% In:
%   M  - Mean state estimate after prediction step
%   P  - State covariance after prediction step
%   Y  - Measurement vector.
%   h  - Measurement model function as a matrix H defining
%        linear function h(x) = H*x, inline function,
%        function handle or name of function in
%        form h(x,param)
%   R  - Measurement covariance.
%   h_param - Parameters of h               (optional, default empty)
%   alpha - Transformation parameter      (optional)
%   beta  - Transformation parameter      (optional)
%   kappa - Transformation parameter      (optional)
%   mat   - If 1 uses matrix form         (optional, default 0)

% Out:
%   M  - Updated state mean
%   P  - Updated state covariance
%   K  - Computed Kalman gain
%   MU - Predictive mean of Y
%   S  - Predictive covariance Y
%   LH - Predictive probability (likelihood) of measurement.


function [M,P,K,MU,S,LH] = updateUnscentedKalman(M,P,Y,h,R,h_param,alpha,beta,kappa,mat)

    %
    % Check that all arguments are there
    %
    if nargin < 5
        error('Too few arguments');
    end
    if nargin < 6
        h_param = [];
    end
    if nargin < 7
        alpha = [];
    end
    if nargin < 8
        beta = [];
    end
    if nargin < 9
        kappa = [];
    end
    if nargin < 10
        mat = [];
    end

    %
    % Apply defaults
    %
    if isempty(mat)
        mat = 0;
    end

    %
    % Do transform and make the update
    %
    tr_param = {alpha beta kappa mat};
    [MU,S,C] = unscentedTransformation(M,P,h,h_param,tr_param);

    S = S + R;
    K = C / S;
    M = M + K * (Y - MU);
    P = P - K * S * K';

    if nargout > 5
        LH = gauss_pdf(Y,MU,S);
    end