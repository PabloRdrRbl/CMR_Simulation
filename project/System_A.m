% Parameters
% x - state vector
% u - input vector
% g - gravity
function A = System_A(x, u, g)
    % TODO: implement the matrix A = df/dx of the continuous linearized 
    % system function dx = f(x, u)
    % Hint: consider using the Symbolic Math Toolbox!
    
    A = zeros(10, 10);
    
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
    
    A = [ 0, 0, 0, 1, 0, 0,                                          0,                                          0,                                          0,                                          0;
          0, 0, 0, 0, 1, 0,                                          0,                                          0,                                          0,                                          0;
          0, 0, 0, 0, 0, 1,                                          0,                                          0,                                          0,                                          0;
          0, 0, 0, 0, 0, 0, 2*B_a_IBx*q1 + 2*B_a_IBy*q2 + 2*B_a_IBz*q3, 2*B_a_IBy*q1 - 2*B_a_IBx*q2 + 2*B_a_IBz*q4, 2*B_a_IBz*q1 - 2*B_a_IBx*q3 - 2*B_a_IBy*q4, 2*B_a_IBx*q4 - 2*B_a_IBy*q3 + 2*B_a_IBz*q2;
          0, 0, 0, 0, 0, 0, 2*B_a_IBx*q2 - 2*B_a_IBy*q1 - 2*B_a_IBz*q4, 2*B_a_IBx*q1 + 2*B_a_IBy*q2 + 2*B_a_IBz*q3, 2*B_a_IBx*q4 - 2*B_a_IBy*q3 + 2*B_a_IBz*q2, 2*B_a_IBx*q3 - 2*B_a_IBz*q1 + 2*B_a_IBy*q4;
          0, 0, 0, 0, 0, 0, 2*B_a_IBx*q3 - 2*B_a_IBz*q1 + 2*B_a_IBy*q4, 2*B_a_IBy*q3 - 2*B_a_IBx*q4 - 2*B_a_IBz*q2, 2*B_a_IBx*q1 + 2*B_a_IBy*q2 + 2*B_a_IBz*q3, 2*B_a_IBy*q1 - 2*B_a_IBx*q2 + 2*B_a_IBz*q4;
          0, 0, 0, 0, 0, 0,                                          0,                              B_omega_IBz/2,                             -B_omega_IBy/2,                              B_omega_IBx/2;
          0, 0, 0, 0, 0, 0,                             -B_omega_IBz/2,                                          0,                              B_omega_IBx/2,                              B_omega_IBy/2;
          0, 0, 0, 0, 0, 0,                              B_omega_IBy/2,                             -B_omega_IBx/2,                                          0,                              B_omega_IBz/2;
          0, 0, 0, 0, 0, 0,                             -B_omega_IBx/2,                             -B_omega_IBy/2,                             -B_omega_IBz/2,                                          0];



end
