clc
clear

% Gravity vector
syms g

% x vector
syms N_r_NBx N_r_NBy N_r_NBz
syms N_v_NBx N_v_NBy N_v_NBz
syms q1 q2 q3 q4
syms N_r_NB N_v_NB q_NB
syms x


N_r_NB = [N_r_NBx; N_r_NBy; N_r_NBz];
N_v_NB = [N_v_NBx; N_v_NBy; N_v_NBz];
q_NB = [q1; q2; q3; q4];
x = [N_r_NBx; N_r_NBy; N_r_NBz; N_v_NBx; N_v_NBy; N_v_NBz; q1; q2; q3; q4];

% x_dot vector
syms N_a_NBx N_a_NBy N_a_NBz
syms q1_dot q2_dot q3_dot q4_dot
syms N_a_NB q_NB_dot
syms x_dot

N_a_NB = [N_a_NBx; N_a_NBy; N_a_NBz];
%q_NB_dot = [q1_dot; q2_dot; q3_dot; q4_dot];
x_dot = sym(zeros(10, 1));

% u vector
syms B_a_IBx B_a_IBy B_a_IBz
syms B_omega_IBx B_omega_IBy B_omega_IBz
syms B_a_IB B_omega_IB
syms u

B_a_IB = [B_a_IBx; B_a_IBy; B_a_IBz];
B_omega_IB = [B_omega_IBx; B_omega_IBy; B_omega_IBz];
u = [B_a_IBx B_a_IBy B_a_IBz B_omega_IBx B_omega_IBy B_omega_IBz];

% Rotation matrix of {B} wrt. {N}
R_NB = [q1^2-q2^2-q3^2+q4^2,      2*(q1*q2-q3*q4),       2*(q1*q3+q2*q4);
            2*(q1*q2+q3*q4), -q1^2+q2^2-q3^2+q4^2,       2*(q2*q3-q1*q4);
            2*(q1*q3-q2*q4),      2*(q2*q3+q1*q4), -q1^2-q2^2+q3^2+q4^2];

R_BN = R_NB';

% Quaternion dynamics (see Slide 2 from "Hilfsblatt")
B_Q_IB = [           0,  B_omega_IBz, -B_omega_IBy, B_omega_IBx;
          -B_omega_IBz,            0,  B_omega_IBx, B_omega_IBy;
           B_omega_IBy, -B_omega_IBx,            0, B_omega_IBz;
          -B_omega_IBx, -B_omega_IBy, -B_omega_IBz,           0];

q_NB_dot = 0.5 * B_Q_IB * q_NB;
q1_dot = q_NB_dot(1); 
q2_dot = q_NB_dot(2); 
q3_dot = q_NB_dot(3);
q4_dot = q_NB_dot(4);

N_a_IB = R_NB * B_a_IB;
N_a_IB(3) = N_a_IB(3) - g;

x_dot(1) = N_v_NBx;
x_dot(2) = N_v_NBy;
x_dot(3) = N_v_NBz;
x_dot(4) = N_a_IB(1); 
x_dot(5) = N_a_IB(2); 
x_dot(6) = N_a_IB(3); 
x_dot(7) = q1_dot;
x_dot(8) = q2_dot;
x_dot(9) = q3_dot;
x_dot(10) = q4_dot;

% A = df/dx
jacobian(x_dot, x);

% B = df/du
jacobian(x_dot, u);

% n - position of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
syms nx ny nz
syms n

n = [nx; ny; nz];

% q - attitude of the NED-Navigation frame {N} frame w.r.t. ECEF-frame {E}
syms q1_EN q2_EN q3_EN q4_EN
syms q_EN

q_EN = [q1_EN; q2_EN; q3_EN; q4_EN];

% Rotation matrix of {N} wrt. {E}
R_EN = [q1_EN^2-q2_EN^2-q3_EN^2+q4_EN^2,                  2*(q1*q2-q3*q4),       2*(q1_EN*q3_EN+q2_EN*q4_EN);
               2*(q1_EN*q2_EN+q3*q4_EN), -q1_EN^2+q2_EN^2-q3_EN^2+q4_EN^2,       2*(q2_EN*q3_EN-q1_EN*q4_EN);
            2*(q1_EN*q3_EN-q2_EN*q4_EN),      2*(q2_EN*q3_EN+q1_EN*q4_EN), -q1_EN^2-q2_EN^2+q3_EN^2+q4_EN^2];

% Transform to ECEF-frame
y = R_EN * N_r_NB + n;

% C = dg/dx
jacobian(y, x)