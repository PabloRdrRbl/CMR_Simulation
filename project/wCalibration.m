function [w_sigma, w_mean, w_bias] = wCalibration (w)
    %
    % w: (3,n) vector containing the angular rates for n time steps

    % Implementation of the sigma/mean/bias computation.
    % The result is a (3,1) vector for each one, containing the value for
    % the three spatial directions
    
    % Operations computed along the second dimension of the matrix.
    % i.e. mean is computed for all the values of w_x along time
    w_sigma = std(w, 0, 2);
    w_mean = mean(w, 2);
    
    % The usual signal of w would be the angular rates of the mobile
    % platform. There are two sources of error: 1. the withe gaussian noise
    % (mean=0 and sigma=1) and second the constant bias.
    % When the mobile platform is at rest and no bias is present the 
    % distribution of the meassurements is a standard normal distribution.
    % The bias causes a displacement of the distribution, moving the mean.
    % As a result, the bias of a rate gyro is the average output when the 
    % device is sitting still.
    w_bias = w_mean;
    
end