function dx = ode_Linear_WT(t,x,info)
%% linearized wind turbine + controller 

LWT = info.sysparam;
disturbance = info.dist;
R = info.setpoint;
F = info.controller;
% R = info.w 
% dx = (LWT.A+LWT.B*F)*x + LWT.B*R + LWT.B_d*disturbance;
w = info.w ;
G = Sylv_solver(LWT,F);

U = F*x + R;
U = F*x + G*w;
dx = LWT.A*x + LWT.B*U + LWT.B_d*disturbance;
