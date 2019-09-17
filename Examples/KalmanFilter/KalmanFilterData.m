function [inputs] =  KalmanFilterData()

    inputs.parameters.alpha = []; inputs.parameters.kappa = []; inputs.parameters.beta = []; 
    inputs.function = @(x,u) x + log(u); %syms x; syms u;


    m = 1000;
    b = 50;

    A = -b/m;
    B = 1/m;
    C = 1;
    D = 0;

    cruise_ss = ss(A,B,C,D);


    [inputs.F, inputs.B, inputs.H, inputs.D] = ssdata(cruise_ss);


    %{
    inputs.F = -0.05;           %state-transition model
    inputs.B = 1.0000e-03;      %control-input model, for each time-step
    inputs.H = 1;               %observation model
    %inputs.D = 0;       
    %}

    inputs.realxstart = 1;
    inputs.predxstart = 0;

    inputs.Qk = 0.5;  %covariance of the process noise
    inputs.Rk = 0.01; %covariance of the observation noise

    inputs.u = 1;               

    %disturbances
    inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
    inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);


    inputs.P = eye(1);          %error covariance


    inputs.param = [];