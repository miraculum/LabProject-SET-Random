function [] = CallKalmanFilter(inputs)

    calculate_RMSE=@(a,b) sqrt(mean((a(:)-b(:)).^2));
    iterations = 500;   xerror = zeros(1, iterations);  
    truexvalues = zeros(1, iterations); predxvalues = zeros(1, iterations);
    xdifference = zeros(1, iterations);
    
    %Kalman Filter
    fprintf('Iteration = 1/%d\n',iterations);
    
    %True X
    [truex, z] = KFTrueX(inputs.F, inputs.realxstart, inputs.B,... 
    inputs.u, inputs.w, inputs.H, inputs.v);

    %Predicted X
    [predx, predp, predy] = KFPredictedX(inputs.P,inputs.F,...
    inputs.predxstart, inputs.B, inputs.u, inputs.Qk, inputs.H, inputs.Rk, z);

    %RMSE Error 
    xerror(1) = calculate_RMSE(truex, predx);
    %Save true X and predicted X
    truexvalues(1) = truex; predxvalues(1) = predx;

    xdifference(1) = truex - predx;

    %Cycle for all iterations
    for i = 1:(iterations-1)
        fprintf('Iteration = %d/%d\n',i+1,iterations);
        
        %Disturbance
        inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
        inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);

        %True X
        [truex, z] = KFTrueX(inputs.F, truex, inputs.B,... 
        inputs.u, inputs.w, inputs.H, inputs.v);

        %Predicted X
        [predx, predp, predy] = KFPredictedX(predp, inputs.F, predx,...
            inputs.B, inputs.u, inputs.Qk, inputs.H, inputs.Rk, z);

        %RMSE Error
        xerror(i+1) = calculate_RMSE(truex, predx);
        
        truexvalues(i+1) = truex;
        predxvalues(i+1) = predx;
        
        xdifference(i+1) = truex - predx;
    end

    figure('Name','Kalman Filter');
    plot(1:iterations, xdifference);
    legend('True X - Predicted X')
    title('Kalman Filter: Subtraction','FontSize',14);
    %ylim([-0.5 1]);
    
    figure('Name','Kalman Filter');
    plot(1:iterations,truexvalues,'b', 1:iterations,predxvalues,'r');
    legend('true x','predicted x');
    title('True x vs Predicted x    Kalman Filter','FontSize',14);
    
    figure('Name','Kalman Filter');
    plot(1:iterations, xerror);
    legend('rmse error')
    title('Kalman Filter: RMSE error','FontSize',14);
    