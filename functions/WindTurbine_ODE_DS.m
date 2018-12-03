function x_s =  WindTurbine_ODE_DS(x_0,t_s,u_s,d_s,Parameter)
% DS on 30-Jul-2015
% local variables ---------------------------------------------------------
dt              = Parameter.Time.dt;
% -------------------------------------------------------------------------

% RK4
nSim            = length(t_s)-1;
x_s             = NaN(length(x_0),nSim+1);
x_s(:,1)        = x_0;


for iSim = 1:nSim
    
    x           = x_s(:,iSim);
    
	% current values    
    u_c         = u_s(:,iSim);      % current
    u_n         = u_s(:,iSim+1);    % next
    u_a         = (u_c+u_n)/2;      % average
    d_c         = d_s(iSim);        % current
    d_n         = d_s(iSim+1);      % next
    d_a         = (d_c+d_n)/2;      % average

	k1         	= state_eqs(x,             u_c,   d_c,  Parameter);
    k2       	= state_eqs(x + 1/2*k1*dt, u_a,   d_a,  Parameter);
    k3       	= state_eqs(x + 1/2*k2*dt, u_a,   d_a,  Parameter);
    k4        	= state_eqs(x +     k3*dt, u_n,   d_n,  Parameter);

    % Update
    x_s(:,iSim+1) = x + 1/6*dt*(k1 + 2*k2 + 2*k3 + k4);

end

end


function dx = state_eqs(x,u,d,Parameter)
% local variables ---------------------------------------------------------
TBN         = Parameter.Turbine;
v_wind      = d;
M_g_dot     = u(1);
theta_c     = u(2);
Omega       = x(1);          	% Hub's rotational speed 
x_T         = x(2);          	% Tower displacement 
x_T_dot     = x(3);            	% Tower displacement speed
theta       = x(4);           	% Pitch Angle
theta_dot   = x(5);           	% Pitch Angle rate
M_g         = x(6);             % generator torque
% -------------------------------------------------------------------------

% Aerodynamics
M_a         = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
F_a         = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);

% Allocation
nx          = Parameter.NMPC.nx;
dx          = zeros(nx,1);    % a column vector

% Equations
dx(1)       = 1/TBN.J       * ( M_a - M_g/TBN.i);
dx(2)       = x_T_dot;
dx(3)       = 1/TBN.m_Te    * ( F_a - TBN.c_Te*x_T_dot - TBN.k_Te*(x_T-TBN.x_T0));
dx(4)       = theta_dot;
dx(5)       = - 2* TBN.xi * TBN.omega * theta_dot - (theta - theta_c) * TBN.omega^2 ;
dx(6)       = M_g_dot;
end
%% end of main function   

%%%%%%%%%%%%%%%%%%%%%%%% Sub Functions  %%%%%%%%%%%%%%%%%%%%%%%>>>
%% Aerodynamic Torque
function M_a = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter)
% local variables ---------------------------------------------------------
TBN     = Parameter.Turbine;
% -------------------------------------------------------------------------
v_rel   = v_wind - x_T_dot;   % relative speed of tower and wind
lambda  = Omega * TBN.R / v_rel;
% c_P     = CalculatePowerCoefficient(lambda,theta);
c_P     = QuickInterp2(Parameter.Turbine.SS.theta,Parameter.Turbine.SS.lambda,Parameter.Turbine.SS.c_P,theta,lambda);

M_a     = 1/2 * TBN.rho * pi * TBN.R^3 * c_P / lambda * v_rel^2;
end



%% Aerodynamic Thrust
function F_a = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter)
% local variables ---------------------------------------------------------
TBN     = Parameter.Turbine;
% -------------------------------------------------------------------------
v_rel   = v_wind - x_T_dot;   % relative speed of tower and wind
lambda  = Omega * TBN.R / v_rel;
% c_T     = CalculateThrustCoefficient(lambda,theta);
c_T     = QuickInterp2(Parameter.Turbine.SS.theta,Parameter.Turbine.SS.lambda,Parameter.Turbine.SS.c_T,theta,lambda);

F_a     = 1/2 * TBN.rho * pi * TBN.R^2 *c_T * v_rel^2;
end


