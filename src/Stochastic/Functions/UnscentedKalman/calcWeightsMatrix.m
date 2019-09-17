%UT_MWEIGHTS - Generate matrix form unscented transformation weights
%
% Syntax:
%   [WM,W,c] = ut_mweights(n,alpha,beta,kappa)
%
% In:
%   n     - Dimensionality of random variable
%   alpha - Transformation parameter  (optional, default 0.5)
%   beta  - Transformation parameter  (optional, default 2)
%   kappa - Transformation parameter  (optional, default 3-size(X,1))
%
% Out:
%   WM - Weight vector for mean calculation
%    W - Weight matrix for covariance calculation
%    c - Scaling constant
%
% Description:
%   Computes matrix form unscented transformation weights.


function [WM,W,c] = calcWeightsMatrix(n,alpha,beta,kappa)

  %
  % Check arguments 
  %
  if nargin < 1
    errorMessage('calcWeightsMatrix: At least dimensionality n required.');
    return;
  end
  if nargin < 2
    alpha = [];
  end
  if nargin < 3
    beta = [];
  end
  if nargin < 4
    kappa = [];
  end
  
  [WM,WC,c] = calcWeights(n,alpha,beta,kappa);

  W = eye(length(WC)) - repmat(WM,1,length(WM));
  W = W * diag(WC) * W';