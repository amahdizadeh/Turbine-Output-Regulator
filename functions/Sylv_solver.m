function out = Sylv_solver(LWT,F)
% Solution of the regulator equations
A = LWT.A; B = LWT.B; C = LWT.C ; D = LWT.D; B_d = LWT.B_d;

A2=[eye(5) zeros(5,2); zeros(2,7)];
A1=[A B; C D];
%  R=-[E;Dew];
% w = [1; Yss];
L1 = 1;
L2 = eye(2);
S=[0   0   0;
    0   0   0;
    0   0   0];
% S=zeros(3,3);

Ew = [B_d*L1, zeros(5,2)];
Dew = [zeros(2,1), -L2];
R=-[Ew;Dew];
 
 
n=5;
m=2;
X=dlyap(inv(A1)*A2,S,inv(A1)*R);
Pi=X(1:n,:);
Gamma=X(n+1:n+m,:);
G = Gamma - F * Pi;
% G*w;
out = G;
% Pi*S-A*Pi-B*Gamma-E,
% C*Pi+D*Gamma+Dew,
% G=Gamma-F*Pi;