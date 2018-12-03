function [fval] = costfn_WT_R3_DS(s,x_0,u_0,t_s,d_s,Parameter)
% DS on 30-Jul-2015
% local variables ---------------------------------------------------------
dt          = Parameter.NMPC.dt;
N           = Parameter.NMPC.N;
% -------------------------------------------------------------------------
% Update Trajectory
U(1,:)      = [u_0(1) s(1:N)'    *Parameter.NMPC.u_cond(1)];
U(2,:)      = [u_0(2) s(N+1:2*N)'*Parameter.NMPC.u_cond(2)];


% Simulation 
nMinorSteps = dt/Parameter.Time.dt;
M           = repmat(2:N+1,nMinorSteps,1);
Idx         = [1;M(:)];

u_s     	= U(:,Idx);
x_s         = WindTurbine_ODE_DS(x_0,t_s,u_s,d_s,Parameter);    
X           = x_s(:,1:nMinorSteps:N*nMinorSteps+1);    
D           = d_s(:,1:nMinorSteps:N*nMinorSteps+1); 

% Cost Function
fval        = CalculateCostFunction(X,U,D,Parameter);

end