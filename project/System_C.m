function C = System_C(x, n, q)
    % Implemetation of the matrix C = dg/dx of the continuous linearized 
    % output function y = g(x)
    % Parameters
    % x - state vector
    % n - position of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
    % q - attitude of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}

    C = zeros(3, 10);
    
    N_r_NBx = x(1);
    N_r_NBy = x(2);
    N_r_NBz = x(3);
    
    nx = n(1);
    ny = n(2);
    nz = n(3);
    
    q1_EN = q(1);
    q2_EN = q(2);
    q3_EN = q(3);
    q4_EN = q(4);
    
    C = [q1_EN^2 - q2_EN^2 - q3_EN^2 + q4_EN^2,                      2*q1*q2 - 2*q3*q4,           2*q1_EN*q3_EN + 2*q2_EN*q4_EN, 0, 0, 0, 2*N_r_NBy*q2, 2*N_r_NBy*q1,   -2*N_r_NBy*q4, -2*N_r_NBy*q3;
                    2*q3*q4_EN + 2*q1_EN*q2_EN, -q1_EN^2 + q2_EN^2 - q3_EN^2 + q4_EN^2,           2*q2_EN*q3_EN - 2*q1_EN*q4_EN, 0, 0, 0,            0,            0, 2*N_r_NBx*q4_EN,             0;
                 2*q1_EN*q3_EN - 2*q2_EN*q4_EN,          2*q1_EN*q4_EN + 2*q2_EN*q3_EN, - q1_EN^2 - q2_EN^2 + q3_EN^2 + q4_EN^2, 0, 0, 0,            0,            0,               0,             0];

end
