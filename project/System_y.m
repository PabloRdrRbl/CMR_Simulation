function y = System_y(x, n, q)
    % Implementation of the nonlinear output function y = g(x)
    %
    % Parameters
    % x - state vector
    % n - position of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
    % q - attitude of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
    %
    % This does not come from the GPS, but is the result to be compared
    % with the GPS measurements within the filter. Vector y is the
    % estimated position of the platform E_y_EB
     
    % Position vector of the platform
    N_r_NB = x(1:3);
    
    % Rotation matrix of {N} to {E}
    R_EN = Quat2DCM(q);
    
    % Transform to ECEF-frame
    E_r_NB = (R_EN * N_r_NB) + n;
   
    % To be compared with the values from GPS, also transformed to ECEF
    y = E_r_NB;

end