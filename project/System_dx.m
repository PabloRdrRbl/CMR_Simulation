function dx = System_dx(t, x, u, g)
    % Implementation of the nonlinear continous system function 
    % dx = f(x, u)
    %
    % Parameters
    % t - current time in seconds
    % x - state vector
    % u - input vector
    % g - gravity
    
    % From system state
    % N_r_NB = [x(1); x(2); x(3)];
    N_v_NB = [x(4); x(5); x(6)];
    
    % From input u
    B_a_IB = [u(1); u(2); u(3)];
    B_omega_IBx = u(4);
    B_omega_IBy = u(5);
    B_omega_IBz = u(6); 
    
    % Initialize nonlinear continous system function dx = f(x, u)
    dx = zeros(10, 1);
    
    % NB-quaternion is part of the system state, from it we can obtain
    % rotation matrix of {B} wrt. {N}
    q1 = x(7);
    q2 = x(8);
    q3 = x(9);
    q4 = x(10);
    q_NB = [q1; q2; q3; q4];
    
    % Rotation matrix of {B} wrt. {N} from quaternion NB
    % From B to N
    R_NB = Quat2DCM(q_NB);
   
    % Transfor acceleration from IMU meassurements to system acceleration
    
    % Quaternion dynamics (see Slide 2 from "Hilfsblatt")
    B_Q_IB = [           0,  B_omega_IBz, -B_omega_IBy, B_omega_IBx;
              -B_omega_IBz,            0,  B_omega_IBx, B_omega_IBy;
               B_omega_IBy, -B_omega_IBx,            0, B_omega_IBz;
              -B_omega_IBx, -B_omega_IBy, -B_omega_IBz,           0];
    
    % Time derivatives of the quaternions
    q_NB_dot = 0.5 * B_Q_IB * q_NB;
    q1_dot = q_NB_dot(1);
    q2_dot = q_NB_dot(2);
    q3_dot = q_NB_dot(3);
    q4_dot = q_NB_dot(4);
     
    % I is equal to E, since the Earth rotation is neglected
    % E is the an inertial frame, and give that NED is a fixed system
    % to the rotating Earth NED can be cosidered an inertial system.
    % As a result the imput accelerations provided IB is can be transformed
    % to NB with a rotation form I (or E) to N.
    % From B to N
    N_a_IB = R_NB * B_a_IB;
    
    % It is necessary to substract the gravity acceleration, in NED this is
    % the down component (z)
    N_a_IB(3) = N_a_IB(3) + abs(g);
    
    dx(1) = N_v_NB(1);
    dx(2) = N_v_NB(2);
    dx(3) = N_v_NB(3);
    
    dx(4) = N_a_IB(1); 
    dx(5) = N_a_IB(2); 
    dx(6) = N_a_IB(3);  
    
    dx(7) = q1_dot;
    dx(8) = q2_dot;
    dx(9) = q3_dot;
    dx(10) = q4_dot;
    
end