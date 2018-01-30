function C = System_C(x, n, q)
    % Implemetation of the matrix C = dg/dx of the continuous linearized 
    % output function y = g(x)
    % Parameters
    % x - state vector
    % n - position of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
    % q - attitude of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}

    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    
    
    C = ...
        [ q1^2 - q2^2 - q3^2 + q4^2,           2*q1*q2 - 2*q3*q4,           2*q1*q3 + 2*q2*q4, 0, 0, 0, 0, 0, 0, 0;
          2*q1*q2 + 2*q3*q4, - q1^2 + q2^2 - q3^2 + q4^2,           2*q2*q3 - 2*q1*q4, 0, 0, 0, 0, 0, 0, 0;
          2*q1*q3 - 2*q2*q4,           2*q1*q4 + 2*q2*q3, - q1^2 - q2^2 + q3^2 + q4^2, 0, 0, 0, 0, 0, 0, 0];
end
