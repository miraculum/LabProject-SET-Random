function [inputs] =  KalmanFilterData()

    %Example 1: 1x1 matrix
    
    m = 1000;
    b = 50;

    A = -b/m;
    B = 1/m;
    C = 1;
    D = 0;

    cruise_ss = ss(A,B,C,D);

    [inputs.F, inputs.B, inputs.H, inputs.D] = ssdata(cruise_ss);

    %Example 2: 2x2 matrix
    %{
    m = 1;
    b = 10;
    k = 20;

    A = [0       1;
    -k/m   -b/m];

    B = [  0;
        1/m];

    C = [1 0];

    D = [0];

    Ts = 1/100;

    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts,'zoh');
    inputs.F = sys_d.A; inputs.B = sys_d.B; inputs.H = sys_d.C; inputs.D = sys_d.D; 
    %}
    
    
    %Exact data, insert below
    %{
    inputs.F = -0.05;           %state-transition model
    inputs.B = 1.0000e-03;      %control-input model, for each time-step
    inputs.H = 1;               %observation model
    inputs.D = 0;               
    %}

    inputs.realxstart = 1;      %real x start position
    inputs.predxstart = 0;      %prediction x start position

    inputs.u = 1;               %control vector

    
    inputs.Qk = 0.05;           %covariance of the process noise
    inputs.Rk = 0.001;          %covariance of the observation noise
    
    
    %Disturbances: Error covariance and Observation noise
    inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
    inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);


    inputs.P = eye(1);          %error covariance