 
function [X, P, y] = KalmanBucy(F, X, P, H, R, u, Q, B, z)

    K = P * H' / R;
    
    X = F * X + B * u + K * (z - H * X);
    P = F * P + P * F' + Q - K * R * K';
    y = z - H * X;