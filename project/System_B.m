% Parameters
% x - state vector
% u - input vector
% g - gravity
function B = System_B(x, u, g)
    % TODO: implement the matrix B = df/du of the continuous linearized 
    % system function dx = f(x, u)
    % Hint: consider using the Symbolic Math Toolbox!
    
    B = zeros(10, 6);
    
        % From system state
    N_r_NBx = x(1);
    N_r_NBy = x(2);
    N_r_NBz = x(3);
    N_v_NBx = x(4);
    N_v_NBy = x(5);
    N_v_NBz = x(6);
    q1 = x(7);
    q2 = x(8);
    q3 = x(9);
    q4 = x(10);
    
    % From input u
    B_a_IBx = u(1);
    B_a_IBy = u(1);
    B_a_IBz = u(3);
    B_omega_IBx = u(4);
    B_omega_IBy = u(5);
    B_omega_IBz = u(6);
    
    B = [                         0,                           0,                           0,     0,     0,     0;
                                  0,                           0,                           0,     0,     0,     0;
                                  0,                           0,                           0,     0,     0,     0;
          q1^2 - q2^2 - q3^2 + q4^2,           2*q1*q2 - 2*q3*q4,           2*q1*q3 + 2*q2*q4,     0,     0,     0;
                  2*q1*q2 + 2*q3*q4, - q1^2 + q2^2 - q3^2 + q4^2,           2*q2*q3 - 2*q1*q4,     0,     0,     0;
                  2*q1*q3 - 2*q2*q4,           2*q1*q4 + 2*q2*q3, - q1^2 - q2^2 + q3^2 + q4^2,     0,     0,     0;
                                  0,                           0,                           0,  q4/2, -q3/2,  q2/2;
                                  0,                           0,                           0,  q3/2,  q4/2, -q1/2;
                                  0,                           0,                           0, -q2/2,  q1/2,  q4/2;
                                  0,                           0,                           0, -q1/2, -q2/2, -q3/2];

end