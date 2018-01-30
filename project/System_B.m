% Parameters
% x - state vector
% u - input vector
% g - gravity
function B = System_B(x, u, g)
    % TODO: implement the matrix B = df/du of the continuous linearized 
    % system function dx = f(x, u)
    % Hint: consider using the Symbolic Math Toolbox!
    
    q1 = x(7);
    q2 = x(8);
    q3 = x(9);
    q4 = x(10);
    
    B = ...
 [                        0,                           0,                           0,     0,     0,     0;
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