function [] = CallUnscentedKalman(inputs, iterations)
    

    calculate_RMSE=@(a,b) sqrt(mean((a(:)-b(:)).^2));
    xerror = zeros(1, iterations);  
    truexvalues = zeros(1, iterations); predxvalues = zeros(1, iterations);
    xdifference = zeros(1, iterations);
    
    
    %Unscented kalman Filter
    fprintf('Iteration = 1/%d\n',iterations);
    
    %True X
    
    [truex] = UKFTrueX(inputs.f, inputs.realxstart, inputs.u, inputs.w);
 

    %function [x, p, f1, h1] =UKFPredictedX(P, f, xkmin1, u, Qk, h, Rk, z, f1, h1)

    [predx, predp] = UKFPredictedX(inputs.predxstart, inputs.P, inputs.f,...
    inputs.Qk, inputs.Rk, inputs.f_param, inputs.alpha, inputs.beta, inputs.kappa, inputs.mat, inputs.Y);

    disp(predx); disp(truex);
    %RMSE Error 
    xerror(1) = norm(calculate_RMSE(truex, predx));
    %Save true X and predicted X
    truexvalues(1) = norm(truex);
    predxvalues(1) = norm(predx);

    xdifference(1) = norm(truex) - norm(predx);


 
     %Cycle for all iterations
    for i = 1:(iterations-1)
        fprintf('Iteration = %d/%d\n',i+1,iterations);
        
        %Disturbance
        inputs.w = mvnrnd(zeros(size(inputs.Qk, 1), 1), inputs.Qk);
        inputs.v = mvnrnd(zeros(size(inputs.Rk, 1), 1), inputs.Rk);

         %True X
         %function [xk] = UKFTrueX(f, xkmin1, u, w)

        truex = UKFTrueX(inputs.f, truex, inputs.u, inputs.w);

        %Predicted X
        [predx] = UKFPredictedX(mean2(predx), inputs.P, inputs.f,...
        inputs.Qk, inputs.Rk, inputs.f_param, inputs.alpha, inputs.beta, inputs.kappa, inputs.mat, inputs.Y);

    
        %RMSE Error
        xerror(i+1) = calculate_RMSE(truex, predx);
        
        truexvalues(i+1) = norm(truex);
        predxvalues(i+1) = norm(predx);
        
        xdifference(i+1) = norm(truex) - norm(predx);
        
        %xdifference(i+1) = truex - predx;
    end

    
    
    figure('Name','Unscented Kalman Filter');
    plot(1:iterations, xerror);
    legend('rmse error')
    title('Kalman Filter: RMSE error','FontSize',14);
    %{
    figure('Name','Kalman Filter');
    plot(1:iterations, xdifference);
    legend('True X - Predicted X')
    title('Kalman Filter: Subtraction','FontSize',14);
    ylim([-0.5 1]);
    
    figure('Name','Kalman Filter');
    plot(1:iterations,truexvalues,'b', 1:iterations,predxvalues,'r');
    legend('true x','predicted x');
    title('True x vs Predicted x    Kalman Filter','FontSize',14);
%}