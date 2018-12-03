function [U,X] = NMPC(U_0,x_0,u_0,t_v,v_0_v,t_0,Parameter)   
% DS on 30-Jul-2015
% local variables ---------------------------------------------------------
dt         	= Parameter.NMPC.dt;
N        	= Parameter.NMPC.N;
LB      	= Parameter.NMPC.LB;
UB        	= Parameter.NMPC.UB;
A         	= [];
B        	= [];
Aeq       	= [];
Beq       	= [];
options     = Parameter.NMPC.options;
% -------------------------------------------------------------------------
% Wind over preview over horizon
t_s         = [0:Parameter.Time.dt:N*dt];
d_s        	= interp1(t_v,v_0_v,t_0+t_s,'linear','extrap');

% start value
s_0         = [U_0(1,2:end)/Parameter.NMPC.u_cond(1),  U_0(2,2:end)/Parameter.NMPC.u_cond(2)]';

% Optimization         
[s,fval,exitflag] = fmincon(@(s) costfn_WT_R3_DS(s,x_0,u_0,t_s,d_s,Parameter),s_0,A,B,Aeq,Beq,LB,UB,...
                            @(s) NLCon_WT(s,x_0,u_0,t_s,d_s,Parameter),options);

% Update Trajectory
U(1,:)      = [u_0(1) s(1:N)'    *Parameter.NMPC.u_cond(1)];
U(2,:)      = [u_0(2) s(N+1:2*N)'*Parameter.NMPC.u_cond(2)];



% Simulation to get the states again 
nMinorSteps = dt/Parameter.Time.dt;
M           = repmat(2:N+1,nMinorSteps,1);
Idx         = [1;M(:)];

u_s     	= U(:,Idx);
x_s        	= WindTurbine_ODE_DS(x_0,t_s,u_s,d_s,Parameter);    
X           = x_s(:,1:nMinorSteps:N*nMinorSteps+1);                      