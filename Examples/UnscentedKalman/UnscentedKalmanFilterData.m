function [inputs] = UnscentedKalmanFilterData()

    inputs.f = @(x,u)(2*u)/(x^2+1);
    inputs.h = @(x)x^2;

    inputs.parameters.alpha = []; inputs.parameters.kappa = []; inputs.parameters.beta = []; 

    %{
    m = 1000;
    b = 50;
    u = 500;

    inputs.xstart = 1;


    s = tf('s');
    P_cruise = 1/(m*s+b);

    Ts = 1/50;

    dP_cruise = c2d(P_cruise,Ts,'zoh');
    [inputs.F, inputs.B, inputs.H ,inputs.D] = ssdata(dP_cruise);
    %}

    m = 1000;
    b = 50;

    A = -b/m;
    B = 1/m;
    C = 1;
    D = 0;

    inputs.realxstart = 1;
    inputs.predxstart = 0;

    cruise_ss = ss(A,B,C,D);
    [inputs.F, inputs.B, inputs.H ,inputs.D] = ssdata(cruise_ss);

    inputs.u = 1;



    inputs.Qk = 0.0005 * 10^2;  %covariance of the process noise
    inputs.Rk = 0.00001 * 10^2; %covariance of the observation noise



    inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
    inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);


    inputs.P = eye(1);
