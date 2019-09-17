function X = calcSigmaPoints(M,P,c)
%   CALCSIGMAPOINTS   Returns sigma points to be used in unscented kalman            
%
%   C = CALCSIGMAPOINTS(M,P,C)
%       
%   See also calcWeights
  [A, p] = chol(P);
  A = A';
  X = [zeros(size(M)) A -A];
  X = sqrt(c)*X + repmat(M,1,size(X,2));