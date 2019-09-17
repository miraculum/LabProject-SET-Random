    % Compute the normal weights 
function [WM,WC,c] = calcWeights(n,alpha,beta,kappa)

% CALCWEIGHTS  Returns weights to be used in unscented kalman and 
%              parameter c to be used to calculate the sigma points.            
%
%   C = CALCWEIGHTS(n,alpha,beta,kappa) n - dimensionality of the random variabe
%       
%   See also calcSigmaPoins
    

    %Check arguments
    if nargin < 1
        errorMessage('calcWeightsMatrix: At least dimensionality n required.');
        return;
    end

    % Apply default values
    
    if isempty(alpha)
      alpha = 1;
    end
    if isempty(beta)
      beta = 0;
    end
    if isempty(kappa)
      kappa = 3 - n;
      
    end
    
    % Compute the normal weights

    lambda = alpha^2 * (n + kappa) - n;

    WM = zeros(2*n+1,1);
    WC = zeros(2*n+1,1);
    for j=1:2*n+1
      if j==1
        wm = lambda / (n + lambda);                             
        wc = lambda / (n + lambda) + (1 - alpha^2 + beta);
      else
        wm = 1 / (2 * (n + lambda));
        wc = wm;
      end
      WM(j) = wm;
      WC(j) = wc;
    end
    
    c = n + lambda;             %Scaling constant