function [N] = System_N (a_sigma, w_sigma)
    % Parameters
    % a_sigma - standard deviation of the accelerometer (3-element vector)
    % w_sigma - standard deviation of the gyroscope (3-element vector)
    
    % Implementation of the input noise covariance matrix
    
    % Input variables are linear acceleration and angular rate estimations.
    % Assuming independence between variables. Data from calibration in
    % Exercise 1a
    N = eye(6);

    N(1,1) = a_sigma(1).^2;
    N(2,2) = a_sigma(2).^2;
    N(3,3) = a_sigma(3).^2;
    N(4,4) = w_sigma(1).^2;
    N(5,5) = w_sigma(2).^2;
    N(6,6) = w_sigma(3).^2;

end