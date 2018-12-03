function [Disturbance,Parameter] = ProcessingConfig
% 30-Jul-2015



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
        
        % Servo 
        Parameter.Turbine.zeta          = 0.7;              % zeta  
        Parameter.Turbine.w             = pi/2;             % omega    
        
            % Cp parameters 4'th order polynomial
        Parameter.Turbine.Cp            =  [ -0.6694;       % p00
                                              0.03285;      % p10
                                              0.3854;       % p01
                                             -0.0007651;    % p20
                                             -0.01064;      % p11
                                             -0.04491;      % p02
                                              0.0003777;    % p21
                                              0.0004907;    % p21
                                              0.002206;     % p03
                                             -8.444e-05;    % p22
                                              1.861e-05;    % p13
                                             -4.366e-05;];  % p04
                                             
                                             
                                             
        %% Controller 
        Parameter.VSC.P_el_rated        = 5e6;              % rated power
        Parameter.VSC.M_g_dot_max       = 15e3;             % maximum torque rate
        Parameter.CPC.Omega_rated       = rpm2radPs(12.1);  % rated rotor speed
        Parameter.CPC.theta_min         = 0; % deg2rad(0);
        Parameter.CPC.theta_max         = pi/2; %deg2rad(90);
        Parameter.CPC.theta_dot_max     = 8 * pi/ 180 ; %deg2rad(8);

        %% Time
        Parameter.Time.dt               = 0.25;
        Parameter.Time.TMax             = 20;
        
        %% Disturbance
         load('Wind_20_2.mat','t','v');
         Disturbance.v_0.time            = t(t>=50&t<=80)-50;
         Disturbance.v_0.signals.values  = v(t>=50&t<=80);
        %Disturbance = 0;        
        %% Initial values
        Omega           = Parameter.CPC.Omega_rated;    
        x_T             = 0.1768;            
        x_T_dot         = 0.000;            
        theta           = deg2rad(17.5237);                   
        M_g             = Parameter.VSC.P_el_rated/Parameter.Turbine.eta/Parameter.CPC.Omega_rated*Parameter.Turbine.i;
        
        x0              = [Omega; x_T ; x_T_dot ; theta; M_g];   % initial value for states        
%         u0              = [0;0];              % initial u0
%         d0              = interp1(Disturbance.v_0.time,Disturbance.v_0.signals.values,0);
        
        Parameter.IC.x_0                = x0;
       


end

