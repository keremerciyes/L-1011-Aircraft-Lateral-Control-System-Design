clc;clear;
syms s k11 k12 k13 k14 k21 k22 k23 k24
A=[0 1 0 0; 0 -1.89 0.39 -5.555; 0 -0.034 -2.98 2.43; 0.034 -0.0011 -0.99 -0.21];
B=[0 0; 0.36 -1.6; -0.95 -0.032; 0.03 0];
C=[1 0 0 0; 0 1 0 0];
D=zeros(size(C,1),size(B,2));
eig(A) %eigens of A
P1 = ctrb(A,B(:,1)); % first input ctrb matrix
P2 = ctrb(A,B(:,2)); % second input ctrb matrix
rank(P1);
rank(P2);
% I choose 2 for first 2 for second
P = [P1(:,1) P1(:,2) P2(:,1) P2(:,2)] %ctrb matrix for Ackermann
rank(P);
P_inv = inv(P) %for Ackermann
eigs=[-2.02506 + 2.02094i; -2.02506 - 2.02094i; -3.4146 + 2.96854i; -3.4146 - 2.96854i];
K = place(A,B,eigs) % this one is MATLAB's defualt func
eig(A-B*K) % cross check
pd=((s-(-2.02506 + 2.02094i))*(s-(-2.02506 - 2.02094i))*(s-(-3.4146 + 2.96854i))*(s-(-3.4146 - 2.96854i)))
pd = coeffs(pd) %desired pol., ans = 167.5624 138.8104 56.3159 10.8793 1.0000
T = [P_inv(2,:);P_inv(2,:)*A;P_inv(4,:);P_inv(4,:)*A] % transformation matrix
A_new = T*A*(inv(T)); % A'
B_new = T*B; % B'
B_new = [0 0; 1 0; 0 0; 0 1];
A_new = [0 1 0 0; -3.082 -3.1639 -0.2717 -0.1507; 0 0 0 1; 2.9056 0.491 0.04 -1.9161];
K_new = [k11 k12 k13 k14; k21 k22 k23 k24]; % K'
prob = A_new + B_new*K_new == [0 1 0 0; 0 0 1 0; 0 0 0 1; -167.5624 -138.8104 -56.3159 -10.8793];
sol = vpasolve(prob);
k11 = double(sol.k11);
k12 = double(sol.k12);
k13 = double(sol.k13);
k14 = double(sol.k14);
k21 = double(sol.k21);
k22 = double(sol.k22);
k23 = double(sol.k23);
k24 = double(sol.k24);
K_new = [k11 k12 k13 k14; k21 k22 k23 k24]
K2 = K_new*T % initial space's static controller
eig(A+B*K2) % cross check