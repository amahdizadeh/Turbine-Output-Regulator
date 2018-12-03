function [out] = Linearize_WT(OperatingPoint, wind_speed, Parameter)



x_T_dot = OperatingPoint(2);
Omega = OperatingPoint(3);
theta = OperatingPoint(4);
v_wind = wind_speed;     


% Calculating Jacobian matrix:

% ================================
%       AERO DYNAMICAL Thrust 
% ================================
%% dF / dXdot
DELTA_x_T_dot = .1;        % 5 m/s / sec
F_a_1 = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);
F_a_2 = CalculateAerodynamicThrust(x_T_dot+DELTA_x_T_dot,Omega,theta,v_wind,Parameter);
DELTA_Fa = F_a_2 - F_a_1;
dF_dXdot = DELTA_Fa / DELTA_x_T_dot;


%% dF / dOMEGA
DELTA_OMEGA = .001   ;     % 5 rad/s / sec
F_a_1 = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);
F_a_2 = CalculateAerodynamicThrust(x_T_dot,Omega+DELTA_OMEGA,theta,v_wind,Parameter);
DELTA_Fa = F_a_2 - F_a_1;
dF_dOMEGA = DELTA_Fa / DELTA_OMEGA;


%% dF / dOMEGA
DELTA_THETA = .001   ;     % 0.01 rad/s / sec
F_a_1 = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);
F_a_2 = CalculateAerodynamicThrust(x_T_dot,Omega,theta+DELTA_THETA,v_wind,Parameter);
DELTA_Fa = F_a_2 - F_a_1;
dF_dTHETA = DELTA_Fa / DELTA_THETA;


%% dF / dWIND
DELTA_WIND = .1   ;     % .1 m/s
F_a_1 = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);
F_a_2 = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind+DELTA_WIND,Parameter);
DELTA_Fa = F_a_2 - F_a_1;
dF_dWIND = DELTA_Fa / DELTA_WIND;

%%
% ================================
%       AERO DYNAMICAL TORQUE 
% ================================
%% dM / dXdot
DELTA_x_T_dot = .01;        % 5 m/s / sec
M_a_1 = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
M_a_2 = CalculateAerodynamicTorque(x_T_dot+DELTA_x_T_dot,Omega,theta,v_wind,Parameter);
DELTA_Ma = M_a_2 - M_a_1;
dM_dXdot = DELTA_Ma / DELTA_x_T_dot;

%% dM / dOMEGA
DELTA_OMEGA = .01;        % 5 m/s / sec
M_a_1 = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
M_a_2 = CalculateAerodynamicTorque(x_T_dot,Omega+DELTA_OMEGA,theta,v_wind,Parameter);
DELTA_Ma = M_a_2 - M_a_1;
dM_dOMEGA = DELTA_Ma / DELTA_OMEGA;


%% dM / dTHETA
DELTA_THETA = .01;        % 5 m/s / sec
M_a_1 = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
M_a_2 = CalculateAerodynamicTorque(x_T_dot,Omega,theta+DELTA_THETA,v_wind,Parameter);
DELTA_Ma = M_a_2 - M_a_1;
dM_dTHETA = DELTA_Ma / DELTA_THETA;


%% dM / dWIND
DELTA_WIND = .025;        % m/s
M_a_1 = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
M_a_2 = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind+DELTA_WIND,Parameter);
DELTA_Ma = M_a_2 - M_a_1;
dM_dWIND = DELTA_Ma / DELTA_WIND;



%% Turbine structural parameters
mTe = Parameter.Turbine.m_Te;
cTe = Parameter.Turbine.c_Te;
kTe = Parameter.Turbine.k_Te;
J = Parameter.Turbine.J;
zeta =   Parameter.Turbine.zeta;
w =   Parameter.Turbine.w;
ig=Parameter.Turbine.i ; % 1/97: gearbox ratio

%  forming A,B,C and D matrices of the linear system
LWT.A = 0;
LWT.A = [     0                  1                    0                     0                   0 ;
          -kTe/mTe    ( -cTe+dF_dXdot)/mTe       dF_dOMEGA/mTe        dF_dTHETA/mTe             0 ;
              0             dM_dXdot/J            dM_dOMEGA/J          dM_dTHETA/J              0 ;
              0                  0                     0                    0                   1 ;
              0                  0                     0                 -w^2               -2*zeta*w];
      
LWT.B = [ 0     0   ;
          0     0   ;
          0   -1/ig/J ;
          0     0   ;
         w^2    0   ];
  
 
LWT.B_d =   [ 0 ; dF_dWIND/mTe ; dM_dWIND/J   ; 0 ; 0 ]        ;


LWT.C = [ 1     0       0      0      0;
          0     0       0      1      0];

  
LWT.D = [ 0  0 ; 0 0 ];


LWT.partials = [dF_dXdot, dF_dOMEGA, dF_dTHETA, dF_dWIND;
               dM_dXdot, dM_dOMEGA, dM_dTHETA, dM_dWIND];
           
out=LWT;           