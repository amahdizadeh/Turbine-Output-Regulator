function [Parameter] = NREL5MWDefaultParameterSLOW
%% General          
Parameter.General.rho                   = 1.225;              % [kg/m^3]  air density

%% Time                     
Parameter.Time.dt                       = 0.0125;           % [s]       time step of simulation

%% Turbine
Parameter.Turbine.HubHeight             = 90; %hub height
Parameter.Turbine.RotorDiameter         = 126; 
Parameter.Turbine.R                     = Parameter.Turbine.RotorDiameter/2;

Parameter.Turbine.i                     = 1/97;
J_G                                     = 534.116;          % [kgm^2]	Generator Inertia About High-Speed Shaft, GenIner from .fst  
J_R                                     = 3.8759e+007;      % [kgm^2]	Generator Rotor About High-Speed Shaft, Rotor Inertia from .fsm
Parameter.Turbine.J                     = J_R+J_G/Parameter.Turbine.i^2;
Parameter.Turbine.eta_el                = 0.944; %GenEff
Parameter.Turbine.eta_gb                = 1; %GBoxEff

Parameter.Turbine.d_s                   = 0.01;             % [-]       Structural Damping ratio from NRELOffshrBsline5MW_Tower_Onshore.dat
Parameter.Turbine.f_0TwFADOF1           = 0.324;
Parameter.Turbine.k_Te                  = 1.81e+06; 
Parameter.Turbine.m_Te                  = Parameter.Turbine.k_Te/(Parameter.Turbine.f_0TwFADOF1*2*pi)^2;
Parameter.Turbine.c_Te                  = 2*Parameter.Turbine.m_Te*Parameter.Turbine.d_s*Parameter.Turbine.f_0TwFADOF1*2*pi;                   % [kg/s]	tower structual damping (sigma=C_T/(2M_T)=D*w_0, w_0=f_0*2*pi, [Gasch] p.294)
Parameter.Turbine.x_T0                  = -0.0140;          % [m]
Parameter.Turbine.DelayAero             = 0.3;              % [s]

%% PitchActuator            
Parameter.PitchActuator.Mode            = 1;                % 0: none; 1: Delay; 2: PT2 (omega, xi, x1_con)
Parameter.PitchActuator.omega           = 2*pi;             % [rad/s]
Parameter.PitchActuator.xi              = 0.7;              % [-]
Parameter.PitchActuator.theta_dot_max 	= deg2rad(8);       % [rad/s]
Parameter.PitchActuator.theta_max     	= deg2rad(90);      % [rad]
Parameter.PitchActuator.theta_min     	= deg2rad(0);       % [rad]

Parameter.PitchActuator.Delay           = 0.2;              % [s]


%% CPC
% Parameter.CPC.SS                        = load('SteadyStatesNREL5MW');  
% Parameter.CPC.SS                        = load('SteadyStatesNREL5MW_SLOW');                    
Parameter.CPC.SS                        = load('SteadyStatesNREL5MW');                    

Parameter.CPC.KP                        = 0.01882681;       % [s]
Parameter.CPC.KI                        = 0.008068634;      % [-]
Parameter.CPC.kp                        = Parameter.CPC.KP;               	% [s]
Parameter.CPC.Ti                        = Parameter.CPC.KP/Parameter.CPC.KI;% [s] 
Parameter.CPC.theta_K                   = deg2rad(6.302336);% [rad]
Parameter.CPC.Omega_rated               = rpm2radPs(12.1);  % [rad/s]
Parameter.CPC.Omega_g_rated             = Parameter.CPC.Omega_rated/Parameter.Turbine.i; % [rad/s]
Parameter.CPC.theta_max                 = deg2rad(90);      % [rad]
Parameter.CPC.theta_min                 = deg2rad(0);       % [rad]

% FF
Parameter.FF.Mode                       = 0;
Parameter.CPC.FF.tau                    = 0.3;              % [s] Parameter.PitchActuator.Delay+Parameter.Turbine.DelayAero+Parameter.Time.dt
Parameter.CPC.FF.dthetadv_0_max         = deg2rad(inf);     % [deg/(m/s)]
Parameter.Scan.T_preview                = 5;                % [s]

%% VSC
Parameter.VSC.Mode                      = 2;                                % 1: ISC, constant power in Region 3; 2: ISC, constant torque in Region 3 

% region limits & region parameters from Jonkman 2009
Parameter.VSC.Omega_g_1To1_5            = rpm2radPs(670);                   % [rad/s]
Parameter.VSC.Omega_g_1_5To2            = rpm2radPs(871);                   % [rad/s]
Parameter.VSC.Omega_g_2_5To3            = rpm2radPs(1161.963);              % [rad/s]
Parameter.VSC.k                         = radPs2rpm(radPs2rpm(0.0255764));  % [Nm/(rad/s)^2]
Parameter.VSC.theta_fine                = deg2rad(1);                       % [rad]
Parameter.VSC.M_g_rated                 = 43093.55150917839;                % [Nm] precision increased by DS to avoid Steady-State-Errors
Parameter.VSC.P_a_rated                 = 5296610.169491526;                % [W]  precision increased by DS to avoid Steady-State-Errors
GeneratorSlipPercentage                 = 0.1;                              % [-]

% Region 1_5: M_g = a * Omega_g + b: 
% 1.Eq: 0                   = a * Omega_g_1To1_5 + b 
% 2.Eq: k*Omega_g_1_5To2^2  = a * Omega_g_1_5To2 + b
Parameter.VSC.a_1_5                     = Parameter.VSC.k*Parameter.VSC.Omega_g_1_5To2^2/(Parameter.VSC.Omega_g_1_5To2-Parameter.VSC.Omega_g_1To1_5);
Parameter.VSC.b_1_5                     = -Parameter.VSC.a_1_5*Parameter.VSC.Omega_g_1To1_5;

% Region 2_5: M_g = a * Omega_g + b: 
% 1.Eq: 0                   = a * Omega_slip        + b 
% 2.Eq: M_g(Omega_g_2_5To3) = a * Omega_g_2_5To3    + b
Omega_slip                              = Parameter.VSC.Omega_g_2_5To3/(1+GeneratorSlipPercentage);
Parameter.VSC.a_2_5                     = (Parameter.VSC.P_a_rated/Parameter.VSC.Omega_g_2_5To3)/(Parameter.VSC.Omega_g_2_5To3-Omega_slip);
Parameter.VSC.b_2_5                     = -Parameter.VSC.a_2_5*Omega_slip;

% intersection k * Omega_g^2 = a * Omega_g + b:
Parameter.VSC.Omega_g_2To2_5            = (Parameter.VSC.a_2_5 - (Parameter.VSC.a_2_5^2+4*Parameter.VSC.k*Parameter.VSC.b_2_5)^.5) / (2*Parameter.VSC.k);

% limits
Parameter.VSC.M_g_dot_max               = 15000;                        % [Nm/s]
Parameter.VSC.M_g_max                   = Parameter.VSC.M_g_rated*1.1;	% [Nm]  
Parameter.VSC.M_g_min                   = 0;                            % [Nm] 

%% Filter                   
Parameter.Filter.FilterGenSpeed.Omega_g.Enable          = 1;
Parameter.Filter.FilterGenSpeed.T63                     = 1/(0.25*2*pi);    % [s], Rise time of PT1 for generator speed filter
Parameter.Filter.FilterSwitchR3.SwitchR3.Enable         = 1;
Parameter.Filter.FilterSwitchR3.T63                     = 5;    % [s]   
