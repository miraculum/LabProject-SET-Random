%TODO: calcular v e I
%      verificar tamadnho das matrizes
%      Há parâmetros que têm de ser passados num vetor
 
function [X, P, Y] = updateKalman(X, P, H, R, z)
    
    %The Kalman filter model assumes the true state at time k 
    %is evolved from the state at (k ? 1) according to
    
        
    %Innovation or measurement pre-fit residual
    Y = z - H * X;
    
    %Innovation (or pre-fit residual) covariance
    S = H * P * H' + R;
    
    %Optimal Kalman gain
    K = P * H' * S^(-1);
    %Updated (a posteriori) state estimate
    X = X + K * Y;
    
    
    %Declaring identity matrix
    [m,n] = size(K * H);
    I = eye(m, n);
    
    %Updated (a posteriori) estimate covariance
    %P = (I - K * H) * P;
    P = P - K*H*P;
    %Measurement post-fit residual
    Y = z - H * X;
