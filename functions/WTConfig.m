function [SimulationName,Disturbance,Parameter] = ProcessingConfig

SimulationName = 'EOGat9d2';  % EOGat9d2   %EOGat20  %HalfCircle

switch SimulationName
    case 'EOGat20'
        
        %% Turbine
        Parameter.Turbine.i             = 1/97;             % gearboox ratio
        Parameter.Turbine.J             = 4.3784e7;         % sum of momentum of inetria about the rotation axis of the rotor hub
        Parameter.Turbine.m_Te          = 4.3675e5;         % tower equivalant modal mass
        Parameter.Turbine.c_Te          = 1.7782e4;         % structural damping coefficient
        Parameter.Turbine.k_Te          = 1810000;          % bending stiffness
        Parameter.Turbine.R             = 63 ;              % the rotor radius
        Parameter.Turbine.rho           = 1.225;            % air density
        Parameter.Turbine.Hh            = 90;               % Hub Height
        Parameter.Turbine.eta           = 0.944;            % efficiency    
        Parameter.Turbine.SS            = load('PowerAndThrustCoefficientsNREL5MW');
        Parameter.Turbine.x_T0      	= -0.0140;          % [m]
      
        %% Controller 
        Parameter.VSC.P_el_rated        = 5e6;              % rated power
        Parameter.VSC.M_g_dot_max       = 15e3;             % maximum torque rate
Parameter.VSC.Mode                      = 1;                                % 1: ISC, constant power in Region 3; 2: ISC, constant torque in Region 3 
        
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

        
        Parameter.CPC.Omega_rated       = rpm2radPs(12.1);  % rated rotor speed
        Parameter.CPC.theta_min         = deg2rad(0);
        Parameter.CPC.theta_max         = deg2rad(90);
        Parameter.CPC.theta_dot_max     = deg2rad(8);
        Parameter.CPC.SS             	= load('SteadyStatesNREL5MW_SLOW');  
        Parameter.CPC.Omega_g_rated             = Parameter.CPC.Omega_rated/Parameter.Turbine.i; % [rad/s]
        
Parameter.CPC.KP                        = 0.01882681;       % [s]
Parameter.CPC.KI                        = 0.008068634;      % [-]
Parameter.CPC.kp                        = Parameter.CPC.KP;               	% [s]
Parameter.CPC.Ti                        = Parameter.CPC.KP/Parameter.CPC.KI;% [s] 
Parameter.CPC.theta_K                   = deg2rad(6.302336);% [rad]        

        %% Time
        Parameter.Time.dt               = 0.25;
        Parameter.Time.TMax             = 30;
        
        %% Disturbance
        load('Wind_20_2.mat','t','v');
        Disturbance.v_0.time            = t(t>=50&t<=95)-50;
        Disturbance.v_0.signals.values  = v(t>=50&t<=95);
                
        %% Initial values
        d0              = interp1(Disturbance.v_0.time,Disturbance.v_0.signals.values,0);
        
        Omega           = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.Omega,d0);                  % x1_0
        x_T             = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.x_T,d0);                                          % x2_0
        x_T_dot         = 0.000;                                           % x3_0         
        theta           = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.theta,d0);                                % x4_0              
        M_g             = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.M_g,d0);     % x5_0
        
        x0              = [Omega; x_T ; x_T_dot ; theta; M_g];             % initial value for states        
        u0              = [0;0];                                           % initial u0
        
        
        Parameter.IC.x_0                = x0;
        Parameter.IC.u_0                = u0;
       
        
        %% NMPC    
        N                               = 8;
        Parameter.NMPC.options          = optimoptions(@fmincon,'Algorithm','sqp','Display','iter','TolFun',1e-4);
        Parameter.NMPC.N                = N;
        Parameter.NMPC.dt               = 0.25;       
        Parameter.NMPC.nx               = length(x0);
        
        Parameter.NMPC.u_cond           = [Parameter.VSC.M_g_dot_max Parameter.CPC.theta_dot_max];        
        Parameter.NMPC.LB               = -ones(2*N,1); 
        Parameter.NMPC.UB               =  ones(2*N,1); 
  

        Parameter.NMPC.COSTFN.Mode      = 'Omega-P_el-x_T-M_g_dot-theta_dot';

        Parameter.NMPC.Q                = [1 1 1];
        Parameter.NMPC.R                = [1e0 1e-3];            
        
        % for Plot
        Parameter.NMPC.nPreview             = 5;        
        Parameter.NMPC.MyColorMatrix        = [ linspace(0,0,Parameter.NMPC.nPreview)' ...
                                                linspace(0,1,Parameter.NMPC.nPreview)'...
                                                linspace(1,1,Parameter.NMPC.nPreview)'];        
        % state boundaries
%         Parameter.NMPC.NONLCON.Mode         = 'none';
         Parameter.NMPC.NONLCON.Mode         = 'Omega_max-theta_min-theta_max';
        Parameter.NMPC.NONLCON.Omega_max 	= 1.2 * Parameter.CPC.Omega_rated;               %  1.2723;
        Parameter.NMPC.NONLCON.theta_min    = Parameter.CPC.theta_min;
        Parameter.NMPC.NONLCON.theta_max    = Parameter.CPC.theta_max;        
   
    case 'HalfCircle'
        
        %% Turbine
        Parameter.Turbine.i             = 1/97;             % gearboox ratio
        Parameter.Turbine.J             = 4.3784e7;         % sum of momentum of inetria about the rotation axis of the rotor hub
        Parameter.Turbine.m_Te          = 4.3675e5;         % tower equivalant modal mass
        Parameter.Turbine.c_Te          = 1.7782e4;         % structural damping coefficient
        Parameter.Turbine.k_Te          = 1810000;          % bending stiffness
        Parameter.Turbine.R             = 63 ;              % the rotor radius
        Parameter.Turbine.rho           = 1.225;            % air density
        Parameter.Turbine.Hh            = 90;               % Hub Height
        Parameter.Turbine.eta           = 0.944;            % efficiency    
        Parameter.Turbine.SS         	= load('PowerAndThrustCoefficientsNREL5MW');
        Parameter.Turbine.x_T0       	= -0.0140;          % [m]
        
        %% Controller 
        Parameter.VSC.P_el_rated        = 5e6;              % rated power
        Parameter.VSC.M_g_dot_max       = 15e3;             % maximum torque rate

Parameter.VSC.Mode                      = 1;                                % 1: ISC, constant power in Region 3; 2: ISC, constant torque in Region 3 
        
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

        
        Parameter.CPC.Omega_rated       = rpm2radPs(12.1);  % rated rotor speed
        Parameter.CPC.theta_min         = deg2rad(0);
        Parameter.CPC.theta_max         = deg2rad(90);
        Parameter.CPC.theta_dot_max     = deg2rad(8);
        Parameter.CPC.SS             	= load('SteadyStatesNREL5MW_SLOW');  
Parameter.CPC.Omega_g_rated             = Parameter.CPC.Omega_rated/Parameter.Turbine.i; % [rad/s]
        
Parameter.CPC.KP                        = 0.01882681;       % [s]
Parameter.CPC.KI                        = 0.008068634;      % [-]
Parameter.CPC.kp                        = Parameter.CPC.KP;               	% [s]
Parameter.CPC.Ti                        = Parameter.CPC.KP/Parameter.CPC.KI;% [s] 
Parameter.CPC.theta_K                   = deg2rad(6.302336);% [rad]        

        %% Time
        Parameter.Time.dt               = 0.25;
        Parameter.Time.TMax             = 20;
        
        %% Disturbance          
        Parameter.TurbSim.URef                  = 12;
        t_down              = 5;
        t_up                = 8;
        T_HalfCircle        = 2;
        Deltav_0_HalfCircle	= 2;
        t_HalfCircle        = 0:Parameter.Time.dt:T_HalfCircle;
        v_0_HalfCircleUp    = Parameter.TurbSim.URef+.5*Deltav_0_HalfCircle*(-1-cos(pi*t_HalfCircle/T_HalfCircle));
        v_0_HalfCircleDown  = Parameter.TurbSim.URef+.5*Deltav_0_HalfCircle*(-1+cos(pi*t_HalfCircle/T_HalfCircle));
        Disturbance.v_0.time                    = [0 t_down+t_HalfCircle t_up+t_HalfCircle Parameter.Time.TMax]';
        Disturbance.v_0.signals.values          = [Parameter.TurbSim.URef  v_0_HalfCircleDown v_0_HalfCircleUp Parameter.TurbSim.URef]';                                


        figure
        plot(Disturbance.v_0.time,Disturbance.v_0.signals.values)
        xlabel('time [s]')
        ylabel('v_0 [m/s]')
                
        %% Initial values
        d0              = interp1(Disturbance.v_0.time,Disturbance.v_0.signals.values,0);
        
        Omega           = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.Omega,d0);                  % x1_0
        x_T             = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.x_T,d0);                                          % x2_0
        x_T_dot         = 0.000;                                           % x3_0         
        theta           = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.theta,d0);                                % x4_0              
        M_g             = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.M_g,d0);     % x5_0
             
        x0              = [Omega; x_T ; x_T_dot ; theta; M_g];             % initial value for states        
        u0              = [0;0];                                           % initial u0
   
        Parameter.IC.x_0                = x0;
        Parameter.IC.u_0                = u0;
        
        %% NMPC    
        N                               = 20;
        Parameter.NMPC.options          = optimoptions(@fmincon,'Algorithm','sqp','Display','iter','TolFun',1e-3);
        Parameter.NMPC.N                = N;
        Parameter.NMPC.dt               = 0.25;
        Parameter.NMPC.nx               = length(x0);

        Parameter.NMPC.u_cond           = [Parameter.VSC.M_g_dot_max Parameter.CPC.theta_dot_max];        
        Parameter.NMPC.LB               = -ones(2*N,1); 
        Parameter.NMPC.UB               =  ones(2*N,1); 

        Parameter.NMPC.COSTFN.Mode      = 'Omega-P_el-x_T-M_g_dot-theta_dot';

        Parameter.NMPC.Q                = [1 1 1e-1];
        Parameter.NMPC.R                = [1e-3 1e-3];         
        
        
        % for Plot
        Parameter.NMPC.nPreview             = 5;        
        Parameter.NMPC.MyColorMatrix        = [ linspace(0,0,Parameter.NMPC.nPreview)' ...
                                                linspace(0,1,Parameter.NMPC.nPreview)'...
                                                linspace(1,1,Parameter.NMPC.nPreview)'];        
        % state boundaries
        Parameter.NMPC.NONLCON.Mode         = 'none';
      
    case 'EOGat9d2'
        
        %% Turbine
        Parameter.Turbine.i             = 1/97;             % gearboox ratio
        Parameter.Turbine.J             = 4.3784e7;         % sum of momentum of inetria about the rotation axis of the rotor hub
        Parameter.Turbine.m_Te          = 4.3675e5;         % tower equivalant modal mass
        Parameter.Turbine.c_Te          = 1.7782e4;         % structural damping coefficient
        Parameter.Turbine.k_Te          = 1810000;          % bending stiffness
        Parameter.Turbine.R             = 63 ;              % the rotor radius
        Parameter.Turbine.rho           = 1.225;            % air density
        Parameter.Turbine.Hh            = 90;               % Hub Height
        Parameter.Turbine.eta           = 0.944;            % efficiency    
        Parameter.Turbine.SS         	= load('PowerAndThrustCoefficientsNREL5MW');
        Parameter.Turbine.x_T0       	= -0.0140;          % [m]
        
        %% Controller 
        Parameter.VSC.P_el_rated        = 5e6;              % rated power
        Parameter.VSC.M_g_dot_max       = 15e3;             % maximum torque rate
        Parameter.VSC.lambda_opt        = 7.55;             % lambda_opt

        Parameter.VSC.Mode              = 1;                                % 1: ISC, constant power in Region 3; 2: ISC, constant torque in Region 3 
        
   
% region limits & region parameters from Jonkman 2009
        Parameter.VSC.Omega_g_1To1_5    = rpm2radPs(670);                   % [rad/s]
        Parameter.VSC.Omega_g_1_5To2    = rpm2radPs(871);                   % [rad/s]
        Parameter.VSC.Omega_g_2_5To3    = rpm2radPs(1161.963);              % [rad/s]
        Parameter.VSC.k                 = radPs2rpm(radPs2rpm(0.0255764));  % [Nm/(rad/s)^2]
        Parameter.VSC.theta_fine        = deg2rad(1);                       % [rad]
        Parameter.VSC.M_g_rated         = 43093.55150917839;                % [Nm] precision increased by DS to avoid Steady-State-Errors
        Parameter.VSC.P_a_rated         = 5296610.169491526;                % [W]  precision increased by DS to avoid Steady-State-Errors
        GeneratorSlipPercentage         = 0.1;                              % [-]

% Region 1_5: M_g = a * Omega_g + b: 
% 1.Eq: 0                   = a * Omega_g_1To1_5 + b 
% 2.Eq: k*Omega_g_1_5To2^2  = a * Omega_g_1_5To2 + b
        Parameter.VSC.a_1_5             = Parameter.VSC.k*Parameter.VSC.Omega_g_1_5To2^2/(Parameter.VSC.Omega_g_1_5To2-Parameter.VSC.Omega_g_1To1_5);
        Parameter.VSC.b_1_5             = -Parameter.VSC.a_1_5*Parameter.VSC.Omega_g_1To1_5;

% Region 2_5: M_g = a * Omega_g + b: 
% 1.Eq: 0                   = a * Omega_slip        + b 
% 2.Eq: M_g(Omega_g_2_5To3) = a * Omega_g_2_5To3    + b
        Omega_slip                      = Parameter.VSC.Omega_g_2_5To3/(1+GeneratorSlipPercentage);
        Parameter.VSC.a_2_5             = (Parameter.VSC.P_a_rated/Parameter.VSC.Omega_g_2_5To3)/(Parameter.VSC.Omega_g_2_5To3-Omega_slip);
        Parameter.VSC.b_2_5             = -Parameter.VSC.a_2_5*Omega_slip;

% intersection k * Omega_g^2 = a * Omega_g + b:
        Parameter.VSC.Omega_g_2To2_5    = (Parameter.VSC.a_2_5 - (Parameter.VSC.a_2_5^2+4*Parameter.VSC.k*Parameter.VSC.b_2_5)^.5) / (2*Parameter.VSC.k);

        
        Parameter.CPC.Omega_rated       = rpm2radPs(12.1);  % rated rotor speed
        Parameter.CPC.theta_min         = deg2rad(0);
        Parameter.CPC.theta_max         = deg2rad(90);
        Parameter.CPC.theta_dot_max     = deg2rad(8);
        Parameter.CPC.SS             	= load('SteadyStatesNREL5MW_SLOW');  
        Parameter.CPC.Omega_g_rated     = Parameter.CPC.Omega_rated/Parameter.Turbine.i; % [rad/s]
        
        Parameter.CPC.KP                = 0.01882681;       % [s]
        Parameter.CPC.KI                = 0.008068634;      % [-]
        Parameter.CPC.kp                = Parameter.CPC.KP;               	% [s]
        Parameter.CPC.Ti                = Parameter.CPC.KP/Parameter.CPC.KI;% [s] 
        Parameter.CPC.theta_K           = deg2rad(6.302336);% [rad]        
        Parameter.CPC.wind_rated       =  11.2;  % m/s

        %% Time
        Parameter.Time.dt               = 0.25;
        Parameter.Time.TMax             = 40;
        
        %% Disturbance          
        load('Wind_9d2.mat','t','v');
        Disturbance.v_0.time            = t(t>=52&t<=100)-52;               % start from t=50 
        Disturbance.v_0.signals.values  = v(t>=52&t<=100);

                
        %% Initial values
        d0              = interp1(Disturbance.v_0.time,Disturbance.v_0.signals.values,0);
        
        Omega           = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.Omega,d0);                  % x1_0
        x_T             = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.x_T,d0);                                          % x2_0
        x_T_dot         = 0.000;                                           % x3_0         
        theta           = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.theta,d0);                                % x4_0              
        M_g             = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.M_g,d0);     % x5_0
             
        x0              = [Omega; x_T ; x_T_dot ; theta; M_g];             % initial value for states        
        u0              = [0;0];                                           % initial u0
   
        Parameter.IC.x_0                = x0;
        Parameter.IC.u_0                = u0;
        
        %% NMPC    
        N                               = 20;
        Parameter.NMPC.options          = optimoptions(@fmincon,'Algorithm','sqp','Display','iter','TolFun',1e-4);
        Parameter.NMPC.N                = N;
        Parameter.NMPC.dt               = 0.25;
        Parameter.NMPC.nx               = length(x0);

        Parameter.NMPC.u_cond           = [Parameter.VSC.M_g_dot_max Parameter.CPC.theta_dot_max];        
        Parameter.NMPC.LB               = -ones(2*N,1); 
        Parameter.NMPC.UB               =  ones(2*N,1); 

%         Parameter.NMPC.COSTFN.Mode      = 'Omega-P_el-x_T-M_g_dot-theta_dot';  % with Omega_rated   
        Parameter.NMPC.COSTFN.Mode      = 'Omega-P_el-x_T-M_g_dot-theta_dot_2';    % with omega_ref

        
        Parameter.NMPC.Q                = [1 1  1 2 ];    % Omega / Mg / xT_dot/ theta
        Parameter.NMPC.R                = [0.1 0.01];     % Mg_dot / theta_dot    
        
        
        % for Plot
        Parameter.NMPC.nPreview             = 5;        
        Parameter.NMPC.MyColorMatrix        = [ linspace(0,0,Parameter.NMPC.nPreview)' ...
                                                linspace(0,1,Parameter.NMPC.nPreview)'...
                                                linspace(1,1,Parameter.NMPC.nPreview)'];        
        % state boundaries
%          Parameter.NMPC.NONLCON.Mode         = 'none';
        Parameter.NMPC.NONLCON.Mode         = 'Omega_max-theta_min-theta_max';

                
        Parameter.NMPC.NONLCON.Omega_max    = 1.2 * Parameter.CPC.Omega_rated;               %  1.2723;
        Parameter.NMPC.NONLCON.theta_min    = Parameter.CPC.theta_min;
        Parameter.NMPC.NONLCON.theta_max    = Parameter.CPC.theta_max;
        
        
end

