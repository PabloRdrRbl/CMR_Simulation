clc
clear all
close all

syms r1 r2 r3 v1 v2 v3 q1 q2 q3 q4
syms a1 a2 a3 w1 w2 w3
syms g

x = [r1; r2; r3; v1; v2; v3; q1; q2; q3; q4];
u = [a1; a2; a3; w1; w2; w3];

a = [a1; a2; a3];
w = [w1; w2; w3];
q = [q1; q2; q3; q4];

R = [q1^2-q2^2-q3^2+q4^2, 2*(q1*q2-q3*q4), 2*(q1*q3+q2*q4);
    2*(q1*q2+q3*q4), -q1^2+q2^2-q3^2+q4^2, 2*(q2*q3-q1*q4);
    2*(q1*q3-q2*q4), 2*(q2*q3+q1*q4), -q1^2-q2^2+q3^2+q4^2];

Q = [0 w3 -w2 w1;
    -w3 0 w1 w2;
    w2 -w1 0 w3;
    -w1 -w2 -w3 0];

a = R * a;
a(3) = a(3) + g;

q = 0.5 * Q * q;

f = [v1; v2; v3; a(1); a(2); a(3); q(1); q(2); q(3); q(4)];  

A = jacobian(f, x);
B = jacobian(f, u);

clc
clear all
close all

syms r1 r2 r3 v1 v2 v3 q1 q2 q3 q4
syms a1 a2 a3 w1 w2 w3
syms g

syms qq1 qq2 qq3 qq4
syms n1 n2 n3

x = [r1; r2; r3; v1; v2; v3; qq1; qq2; qq3; qq4];

r = [r1; r2; r3];
n = [n1; n2; n3];

R = [q1^2-q2^2-q3^2+q4^2, 2*(q1*q2-q3*q4), 2*(q1*q3+q2*q4);
    2*(q1*q2+q3*q4), -q1^2+q2^2-q3^2+q4^2, 2*(q2*q3-q1*q4);
    2*(q1*q3-q2*q4), 2*(q2*q3+q1*q4), -q1^2-q2^2+q3^2+q4^2];

g = (R * r) + n;

C = jacobian(g, x)



