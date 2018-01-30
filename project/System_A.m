% Parameters
% x - state vector
% u - input vector
% g - gravity
function A = System_A(x, u, g)
    % TODO: implement the matrix A = df/dx of the continuous linearized 
    % system function dx = f(x, u)
    % Hint: consider using the Symbolic Math Toolbox!
    
    q1 = x(7);
    q2 = x(8);
    q3 = x(9);
    q4 = x(10);
    a1 = u(1);
    a2 = u(2);
    a3 = u(3);
    w1 = u(4);
    w2 = u(5);
    w3 = u(6);
    
    A = ...
    [ 0, 0, 0, 1, 0, 0,                           0,                           0,                           0,                           0;
      0, 0, 0, 0, 1, 0,                           0,                           0,                           0,                           0;
      0, 0, 0, 0, 0, 1,                           0,                           0,                           0,                           0;
      0, 0, 0, 0, 0, 0, 2*a1*q1 + 2*a2*q2 + 2*a3*q3, 2*a2*q1 - 2*a1*q2 + 2*a3*q4, 2*a3*q1 - 2*a1*q3 - 2*a2*q4, 2*a1*q4 - 2*a2*q3 + 2*a3*q2;
      0, 0, 0, 0, 0, 0, 2*a1*q2 - 2*a2*q1 - 2*a3*q4, 2*a1*q1 + 2*a2*q2 + 2*a3*q3, 2*a1*q4 - 2*a2*q3 + 2*a3*q2, 2*a1*q3 - 2*a3*q1 + 2*a2*q4;
      0, 0, 0, 0, 0, 0, 2*a1*q3 - 2*a3*q1 + 2*a2*q4, 2*a2*q3 - 2*a1*q4 - 2*a3*q2, 2*a1*q1 + 2*a2*q2 + 2*a3*q3, 2*a2*q1 - 2*a1*q2 + 2*a3*q4;
      0, 0, 0, 0, 0, 0,                           0,                        w3/2,                       -w2/2,                        w1/2;
      0, 0, 0, 0, 0, 0,                       -w3/2,                           0,                        w1/2,                        w2/2;
      0, 0, 0, 0, 0, 0,                        w2/2,                       -w1/2,                           0,                        w3/2;
      0, 0, 0, 0, 0, 0,                       -w1/2,                       -w2/2,                       -w3/2,                           0];
    
end
