function [] = CallExtendedKalmanFilter(inputs, iterations)

    %RMSE function
    calculate_RMSE=@(a,b) sqrt(mean((a(:)-b(:)).^2));
    
    xerror = zeros(1, iterations);  
    truexvalues = zeros(1, iterations); predxvalues = zeros(1, iterations);
    xdifference = zeros(1, iterations);
    
    %Derivate functions
    syms symX symY;
    f1 = matlabFunction( diff(inputs.f(symX,symY),symX) );
    h1 = matlabFunction( diff(inputs.h(symX)) );
    
    %Extended Kalman

    %True X
    [truex, z] = EKFTrueX(inputs.f, inputs.h, inputs.realxstart, inputs.u,...
        inputs.w, inputs.v);
    
    %Predicted X
    [predx, predp, f1, h1] = EFKPredictedX(inputs.P, inputs.f,...
        inputs.predxstart, inputs.u, inputs.Qk, inputs.h, inputs.Rk, z, f1, h1);
    %RMSE Error 
    xerror(1) = norm(calculate_RMSE(truex, predx));
    %Save true X and predicted X
    truexvalues(1) = norm(truex); predxvalues(1) = norm(predx);
    
    xdifference(1) = norm(truex) - norm(predx);
    
    %Cycle for all iterations
    for i = 1:(iterations-1)
        fprintf('Iteration = %d/%d\n',i+1,iterations);
        
        %Disturbance
        inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
        inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);

        %True X
        [truex, z] = EKFTrueX(inputs.f, inputs.h, truex, inputs.u,...
            inputs.w, inputs.v);

        %Predicted X
        [predx, predp, f1, h1] = EFKPredictedX(predp, inputs.f, predx,...
            inputs.u, inputs.Qk, inputs.h, inputs.Rk, z, f1, h1);
        
        %RMSE Error
        xerror(i+1) = calculate_RMSE(norm(truex), norm(predx));
        
        truexvalues(i+1) = norm(truex);
        predxvalues(i+1) = norm(predx);
        
        xdifference(i+1) = norm(truex) - norm(predx);
    end

    [numRows,numCols] = size(predx);
    if (numRows==1 && numCols==1)
        figure('Name','Extended Kalman Filter');
        plot(1:iterations, xdifference);
        legend('True X - Predicted X')
        title('Extended Kalman Filter: Subtraction','FontSize',14);
        ylim([-0.5 1]);
    end

    figure('Name','Extended Kalman Filter');
    plot(1:iterations,truexvalues,'b', 1:iterations,predxvalues,'r');
    legend('true x','predicted x');
    title('True x vs Predicted x    Extended Kalman Filter','FontSize',14);
    
    figure('Name','Extended Kalman Filter');
    plot(1:iterations, xerror);
    legend('rmse error')
    title('Extended Kalman Filter: RMSE','FontSize',14);
    