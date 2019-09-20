% Unscented Kalman predict %

% In:
%   x - Nx1 mean state estimate of previous step
%   P - NxN state covariance of previous step
%   f - Dynamic model function as function handle in
%       form f(x,param)                   (optional, default eye())
%   Q - Process noise of discrete model   (optional, default zero)
%   f_param - Parameters of f             (optional, default empty)
%   alpha - Transformation parameter      (optional)
%   beta  - Transformation parameter      (optional)
%   kappa - Transformation parameter      (optional)
%   mat   - If 1 uses matrix form         (optional, default 0)

% Out:
%   x - Updated state mean
%   P - Updated state covariance

% Check:
%  UnscentedTransformation, CalcWeights, CalcWeightsMatrix, CalcSigmaPoints


function [x,P,D] = predictUnscentedKalman(x,P,f,Q,f_param,alpha,beta,kappa,mat)

  % Number of inputs control 
  if nargin < 2
    errorMessage('predictUnscentedKalman needs at least 2 arguments');
    return;
  end
  if nargin < 3
    f = [];
  end
  if nargin < 4
    Q = [];
  end
  if nargin < 5
    f_param = [];
  end
  if nargin < 6
    alpha = [];
  end
  if nargin < 7
    beta = [];
  end
  if nargin < 8
    kappa = [];
  end
  if nargin < 9
    mat = [];
  end

  %
  % Apply defaults
  %
  if isempty(f)
    f = eye(size(x,1));
  end
  if isempty(Q)
    Q = zeros(size(x,1));
  end
  if isempty(mat)
    mat = 0;
  end

  %
  % Transformation
  % Add process noise
  %
  tr_param = {alpha beta kappa mat};
  [x,P,D] = unscentedTransformation(x,P,f,f_param,tr_param);
  P = P + Q;