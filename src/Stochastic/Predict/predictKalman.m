%TODO: verificar tamanho das matrizes
%      Há parâmetros que têm de ser passados num vetor
 
function [x, P] = predictKalman(P, x, F, B, u, Q)

    if isempty(F)
        F = eye(size(x,1));
    end
    
    if isempty(B)
        B = eye(size(x,1),size(u,1));
    end
  
    ft = F';
    
    if isempty(Q)
        Q = eye(size(ft,1));
    end


    %predicted state estimate
    x = F * x + B * u;
         
    %predicted error covariace
    P = F * P * ft + Q;
    

    
