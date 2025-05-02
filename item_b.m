clc;clear;
syms s k11 k12 k13 k14 k21 k22 k23 k24
syms x1 x2 x3 x4
x=[x1;x2;x3;x4];
A=[0 1 0 0; 0 -1.89 0.39 -5.555; 0 -0.034 -2.98 2.43; 0.034 -0.0011 -0.99 -0.21];
B=[0 0; 0.36 -1.6; -0.95 -0.032; 0.03 0];
C=[1 0 0 0; 0 1 0 0];
D=zeros(size(C,1),size(B,2));
K = [0 0 k13 k14; k21 k22 0 0]; % K'
prob = coeffs(det(s*eye(4)-A+B*K),s) == coeffs((s-(2.02506+2.02094i))*(s-(2.02506-2.02094i))* ...
(s-(-3.4146+2.96854i))*(s-(-3.4146-2.96854i)));
sol = vpasolve(prob)
sol.k13(1)
sol.k14(1)
sol.k21(1)
sol.k22(1)
K2 = [0 0 sol.k13(1) sol.k14(1); sol.k21(1) sol.k22(1) 0 0]
eig(A-B*K2) % cross validation