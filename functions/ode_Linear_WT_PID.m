function dx = ode_Linear_WT_PID(t,x,info,Parameter,Disturbance)
%% linearized wind turbine + controller 
dx=zeros(6,1);
LWT = info.LWT;
P = info.P;
I = info.I;
% R = info.w 

omega_err = (0 - x(3))*0.45;

PID = 0.63* omega_err + 0.259 * x(6);   %% Original values for CDC 2016 paper
% PID = 0.1100*(0.0188* omega_err + 0.0081 * x(6));    % values from Schlipf   


trq_rated = Parameter.VSC.P_el_rated/Parameter.CPC.Omega_rated;
omega_act = Parameter.CPC.Omega_rated +  x(3);
trq_act = 5e6 / omega_act;
trq_diff = trq_act - trq_rated;

M_g = -trq_diff / 97 ;
theta_c = -PID - x(2)*-0; 


wind =  interp1(Disturbance.time,Disturbance.values,t);
U = [theta_c ; M_g];
dx(1:5) = LWT.A*x(1:5) + LWT.B*U + LWT.B_d*wind;
dx(6) = omega_err;
