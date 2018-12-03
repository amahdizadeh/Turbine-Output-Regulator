function [err, out] = fitting_error(p,sig,TC,init_x)

N = length(sig);
tf = sig(end,1);
tspan = linspace(0,tf,N);

% calculating x(0) and differentiations 
x0 = smoothed_diffs (sig,length(p)-1,fix(N/10)+1,1);
if (nargin==4)
   for i=1:min(length(init_x),length(p))
       if (init_x(1,i)~=0)
          x0(i) = init_x(2,i);
       end
   end
end
ode_options = odeset('RelTol',1e-4,'AbsTol',ones(length(p),1)* 1e-4);
[Time,y] = ode45(@ode_LinAutoSys,tspan,x0,ode_options,p);   

% returning the discrepency
% x_end = smoothed_diffs (s,length(p)-1,fix(N/10)+1,499-fix(N/10)+1);
err = sum(( y(:,1) - sig(:,2)).^2)/N + TC*(y(end,1) - sig(end,2))^2 + 0/2 * norm(y(end,1:3) - sig(end,2:4));
out = [Time, y(:,1)];



% % Linear Autonomous differential equation
% function dx = ode_generic(t,x,p)
% 
% A = eye(length(p));A = circshift(A,-1);A(end,:)=p';
% 
% dx = A*x;

