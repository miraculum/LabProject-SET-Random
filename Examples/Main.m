%Filter(data, iterations)


                   %%%%%%%%%% Kalman Filter %%%%%%%%%%%%

[inputs] = KalmanFilterData();
CallKalmanFilter(inputs, 300);


                %%%%%%%%% Extended Kalman Filter %%%%%%%%%%


%[inputs] = ExtendedKalmanFilterData();
%CallExtendedKalmanFilter(inputs, 300);


                 %%%%%%%%% Kalman Bucy Filter %%%%%%%%%%%

%[inputs] = KalmanBucyData();
%CallKalmanBucy(inputs, 300);


                %%%%%%%%% Unscented Kalman Filter %%%%%%%%%

%[inputs] = ExtendedKalmanFilterData();
%CallUnscentedKalman(inputs, 300)
