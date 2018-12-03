function [t_s,x_s,u_s] = SimulateBaselineController(x_0_BL,Disturbance,Parameter)

% local variables ---------------------------------------------------------
dt              = Parameter.Time.dt;
N               = Parameter.NMPC.N;
% -------------------------------------------------------------------------

% RK4
t_s             = [0:Parameter.Time.dt:Parameter.Time.TMax];
nSim            = length(t_s)-1;
x_s             = NaN(length(x_0_BL),nSim+1);
x_s(:,1)        = x_0_BL;
d_s             = interp1(Disturbance.v_0.time,Disturbance.v_0.signals.values,t_s,'linear','extrap');

theta_memory    = x_0_BL(4);

for iSim = 1:nSim
    
    x           = x_s(:,iSim);
    
	% current values    
    d_c         = d_s(iSim);        % current
    d_n         = d_s(iSim+1);      % next
    d_a         = (d_c+d_n)/2;      % average

	k1         	= state_eqs(x,                d_c,  theta_memory, Parameter);
    k2       	= state_eqs(x + 1/2*k1*dt,    d_a,  theta_memory, Parameter);
    k3       	= state_eqs(x + 1/2*k2*dt,    d_a,  theta_memory, Parameter);
    k4        	= state_eqs(x +     k3*dt,    d_n,  theta_memory, Parameter);

    % Update
    x_s(:,iSim+1)   = x + 1/6*dt*(k1 + 2*k2 + 2*k3 + k4);
    
    % Update memory based on integrated states
    [~,M_g,theta]   = state_eqs(x,d_c,theta_memory,Parameter); % d does not matter
    u_s(:,iSim)     = [M_g,theta];
    theta_memory    = theta;

end

% last control input
iSim            = nSim+1;
x               = x_s(:,iSim);
[~,M_g,theta]   = state_eqs(x,d_c,theta_memory,Parameter); % d does not matter
u_s(:,iSim)     = [M_g,theta];

end


function [dx,M_g,theta] = state_eqs(x,d,theta_memory,Parameter)
% local variables ---------------------------------------------------------
TBN         = Parameter.Turbine;
v_wind      = d;
Omega       = x(1);          	% Hub's rotational speed 
x_T         = x(2);          	% Tower displacement 
x_T_dot     = x(3);            	% Tower displacement speed
Integrator  = x(4);           	% Intergrator of PI
% -------------------------------------------------------------------------

% Pitch Controller
Omega_g     = Omega/Parameter.Turbine.i;
Error       = Omega_g - Parameter.CPC.Omega_g_rated;
Gain        = 1/(1+theta_memory/Parameter.CPC.theta_K);
theta_unc   = Parameter.CPC.kp*Error*Gain+Integrator;
theta       = min(max(theta_unc,Parameter.CPC.theta_min),Parameter.CPC.theta_max);
AntiWindUp  = theta-theta_unc;

% Torque Controller
M_g         = BaselineVSControl(Omega_g,theta,Parameter);


% Aerodynamics
M_a         = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
F_a         = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);



% Allocation
nx          = length(x);
dx          = zeros(nx,1);    % a column vector

% Equations
dx(1)       = 1/TBN.J       * ( M_a - M_g/TBN.i);
dx(2)       = x_T_dot;
dx(3)       = 1/TBN.m_Te    * ( F_a - TBN.c_Te*x_T_dot - TBN.k_Te*(x_T-TBN.x_T0));
dx(4)       = (Parameter.CPC.kp*Error*Gain+AntiWindUp)/Parameter.CPC.Ti;


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



function M_g = BaselineVSControl(Omega_g,theta,Parameter)
 
Omega_g_1To1_5  = Parameter.VSC.Omega_g_1To1_5;
Omega_g_1_5To2  = Parameter.VSC.Omega_g_1_5To2;
Omega_g_2To2_5  = Parameter.VSC.Omega_g_2To2_5;
Omega_g_2_5To3  = Parameter.VSC.Omega_g_2_5To3;  
a_1_5           = Parameter.VSC.a_1_5;           
b_1_5           = Parameter.VSC.b_1_5;
k               = Parameter.VSC.k;
a_2_5           = Parameter.VSC.a_2_5;
b_2_5           = Parameter.VSC.b_2_5;
M_g_rated       = Parameter.VSC.M_g_rated;
theta_fine      = Parameter.VSC.theta_fine;
P_a_rated       = Parameter.VSC.P_a_rated;
Mode            = Parameter.VSC.Mode;

if      Omega_g_2_5To3  <   Omega_g  || theta >= theta_fine % Region 3
    if      Mode ==1 % Power constant
        M_g = P_a_rated/Omega_g;
    elseif  Mode ==2 % Torque constant
        M_g = M_g_rated;
    else
        M_g = NaN;
    end

elseif  Omega_g_2To2_5  <   Omega_g     % Region 2.5
    M_g = a_2_5 * Omega_g + b_2_5;

elseif  Omega_g_1_5To2  <   Omega_g     % Region 2
    M_g = k * Omega_g^2;

elseif  Omega_g_1To1_5  <   Omega_g     % Region 1.5
    M_g = a_1_5 * Omega_g + b_1_5;

else                                    % Region 1 
    M_g = 0;    
end

end

