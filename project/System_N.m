function [N] = System_N (a_sigma, w_sigma)
    % Parameters
    % a_sigma - standard deviation of the accelerometer (3-element vector)
    % w_sigma - standard deviation of the gyroscope (3-element vector)
    
    % Implementation of the input noise covariance matrix
    
    % Input variables are linear acceleration and angular rate estimations.
    % Assuming independence between variables. Data from calibration in
    % Exercise 1a
    N = eye(6);
    
    sigma_a = [0.201986, 0.251961, 0.411880];
    sigma_w = [0.035174, 0.007314, 0.004723];
    
    N(1,1) = sigma_a(1).^2;
    N(2,2) = sigma_a(2).^2;
    N(3,3) = sigma_a(3).^2;
    N(4,4) = sigma_w(1).^2;
    N(5,5) = sigma_w(2).^2;
    N(6,6) = sigma_w(3).^2;

end