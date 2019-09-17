function [] = CallKalmanBucy(inputs)

    calculate_RMSE=@(a,b) sqrt(mean((a(:)-b(:)).^2));
    iterations = 20;   xerror = zeros(1, iterations);  
    truexvalues = zeros(1, iterations);predxvalues = zeros(1, iterations);
    
    %Kalman Bucy
    fprintf('Iteration = 1/%d\n',iterations);
    
    %True X
    [truex, z] = KBTrueX(inputs.F, inputs.realxstart, inputs.B,... 
    inputs.u, inputs.w, inputs.H, inputs.v);

    %Predicted X
    [predx, predp, predy] = KalmanBucy(inputs.F, inputs.predxstart,...
        inputs.P, inputs.H, inputs.Rk, inputs.u, inputs.Qk, inputs.B, z);

    %RMSE Error 
    xerror(1) = calculate_RMSE(truex, predx);
    %Save true X and predicted X
    truexvalues(1) = truex; predxvalues(1) = predx;
    

    %Cycle for all iterations
    for i = 1:(iterations-1)
        fprintf('Iteration = %d/%d\n',i+1,iterations);
        
        %Disturbance
        inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
        inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);

        %True X
        [truex, z] = KBTrueX(inputs.F, truex, inputs.B,... 
            inputs.u, inputs.w, inputs.H, inputs.v);

        %Predicted X
        [predx, predp, predy] = KalmanBucy(inputs.F, predx,...
        predp, inputs.H, inputs.Rk, inputs.u, inputs.Qk, inputs.B, z);

        %RMSE Error
        xerror(i+1) = calculate_RMSE(truex, predx);
        
        truexvalues(i+1) = truex;
        predxvalues(i+1) = predx;
    end

    
    figure('Name','Kalman Bucy');
    plot(1:iterations,truexvalues,'b', 1:iterations,predxvalues,'r');
    legend('true x','predicted x');
    title('True vs Predicted x  -  Kalman Bucy','FontSize',14);
    
    figure('Name','Kalman Bucy');
    plot(1:iterations, xerror);
    legend('rmse error')
    title('Kalman Bucy: RMSE error','FontSize',14);
    