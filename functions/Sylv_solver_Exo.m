function [out1, out2] = Sylv_solver_Exo(LinearSys,Exo_sys1,Exo_sys2)
% Solution of the regulator equations
A = LinearSys.A; B = LinearSys.B; C = LinearSys.C ; D = LinearSys.D; B_d = LinearSys.B_d;
L1 = Exo_sys1.L;  L2 = Exo_sys2.L;  S1 = Exo_sys1.S;  S2 = Exo_sys2.S;  

n=length(A);                    % system states number
m=size(B,2);                    % system input dimention
p=length(L1);           % order of the disturbance Exo system
q=length(L2);           % order of the Reference Exo system

A2=[eye(n) zeros(n,m); zeros(m,n+m)];
A1=[A B; C D];
%  R=-[E;Dew];



S = blkdiag(S1,S2);

Ew = [B_d*L1, zeros(n,q)];
Dew = [zeros(m,p), -L2];
R=-[Ew;Dew];
 
 

X=dlyap(inv(A1)*A2,S,inv(A1)*R);
Pi=X(1:n,:);
Gamma=X(n+1:n+m,:);
out1 = Pi;
out2 = Gamma;
%% Test    
% Pi*S-A*Pi-B*Gamma-Ew,
% C*Pi+D*Gamma+Dew,
