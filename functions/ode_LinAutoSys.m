function dx = ode_LinAutoSys(t,x,p)
% Nummerically solves ODE of linear Autonomous System with given initial
% values and A matrix by X_dot = A * X0

A = eye(length(p));A = circshift(A,-1);A(end,:)=p';

dx = A*x;