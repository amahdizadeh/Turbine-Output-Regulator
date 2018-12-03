function [out] = Linearize_5MW(type , wind_speed)


if strcmp(type,'FAST')
   load('FAST_LWT_9m_3DOF_Torsion.mat')   
   LinearWT_FAST.B = -LinearWT_FAST.B;
   out = LinearWT_FAST;
   return
end



if strcmp(type,'Wright')
    load('CP_CT_SLOW_NREL5MW.mat')
    load('SteadyStatesValues_NREL5MW.mat');  
%     Parameter                               = NREL5MWDefaultParameterFAST;
    [~,Params]   = ProcessingConfig;
    working_point = [ interp1(SSValues.v_0 , SSValues.x_T , wind_speed), ...
                        0,...
                        interp1(SSValues.v_0 , SSValues.Omega , wind_speed),...
                        0,...
                        0];
                    
    Par = Partials(working_point,wind_speed,Params,Coeffs)   ;
    N=97;
    gamma = Par(2,2);
    alpha     = Par(2,4);

     Kd = 867e6;
     Cd = 6.2e6;

     J_rot = 11776047;
     J_gen = 534*N^2;
     %% State variables
     % x1 : Omega(Rotor)
     % x2 : Torsion
     % x3 : Omega(Gen)
     
%      LWT.A = [(gamma-Cd)/J_rot              -1/J_rot             Cd/J_rot;
%               Kd                               0                      -Kd;
%               Cd/J_gen                       1/J_gen            -Cd/J_gen];
          
     LWT.A = [(gamma-Cd)/J_rot              -Kd/J_rot             Cd/J_rot;
              1                               0                      -1;
              Cd/J_gen                       Kd/J_gen            -Cd/J_gen];          

      LWT.B  = [ 0   0  -1/J_gen*N]';
      LWT.B_d = [ alpha/J_rot 0  0 ]';
      
      LWT.Cy = [ 1 0 0;
                0 0 1];      
      LWT.C = [ 1 0 0];
      LWT.D = 0;
      LWT.Dy = 0;
      out = LWT;
      return
end

if strcmp(type,'Reg2_5DOF')
    load('CP_CT_SLOW_NREL5MW.mat')
    load('SteadyStatesValues_NREL5MW.mat');  
    Parameter                               = NREL5MWDefaultParameterFAST;
    [~,Params]   = ProcessingConfig;
    working_point = [ interp1(SSValues.v_0 , SSValues.x_T , wind_speed), ...
                        0,...
                        interp1(SSValues.v_0 , SSValues.Omega , wind_speed),...
                        0,...
                        0];
                    
    Par = Partials(working_point,wind_speed,Params,Coeffs)   ;
    N=97;
    gamma = Par(2,2);
    alpha     = Par(2,4);

     Kd = 867e6;
     Cd = 6.2e6;

     J_rot = 11776047;
     J_gen = 534*N^2;
     %% State variables
     % x1 : Omega(Rotor)
     % x2 : Torsion
     % x3 : Omega(Gen)
     
%      LWT.A = [(gamma-Cd)/J_rot              -1/J_rot             Cd/J_rot;
%               Kd                               0                      -Kd;
%               Cd/J_gen                       1/J_gen            -Cd/J_gen];
          
     LWT.A = [(gamma-Cd)/J_rot              -Kd/J_rot             Cd/J_rot;
              1                               0                      -1;
              Cd/J_gen                       Kd/J_gen            -Cd/J_gen];          

      LWT.B  = [ 0   0  -1/J_gen*N]';
      LWT.B_d = [ alpha/J_rot 0  0 ]';
      
      LWT.Cy = [ 1 0 0;
                0 0 1];      
      LWT.C = [ 1 0 0];
      LWT.D = 0;
      LWT.Dy = 0;
      out = LWT;
      return     
end

if strcmp(type,'Wright_R3')
    load('CP_CT_SLOW_NREL5MW.mat')
    load('SteadyStatesValues_NREL5MW.mat');  
    Parameter                               = NREL5MWDefaultParameterFAST;
    [~,Params]   = ProcessingConfig;
    working_point = [ interp1(SSValues.v_0 , SSValues.x_T , wind_speed), ...
                        0,...
                        interp1(SSValues.v_0 , SSValues.Omega , wind_speed),...
                        interp1(SSValues.v_0 , SSValues.Theta , wind_speed),...
                        0];
    % partials  dM_dXdot, dM_dOMEGA, dM_dTHETA, dM_dWIND                             
    Par = Partials(working_point,wind_speed,Params,Coeffs)   ;

    gamma = Par(2,2); %dM_dOMEGA
    beta  = Par(2,3); %dM_dTHETA
    alpha = Par(2,4); %dM_dWIND
    
     N=97;
     Kd = 867e6;
     Cd = 6.2e6;

     J_rot = 11776047;
     J_gen = 534*N^2;
     w = Parameter.PitchActuator.omega;
     xi = Parameter.PitchActuator.xi;
     
     %% State variables
     % x1 : Omega(Rotor)
     % x2 : Torsion
     % x3 : Omega(Gen)
     % x4 : Theta
     % x5 : Theta_dot
%      LWT.A = [(gamma-Cd)/J_rot              -1/J_rot             Cd/J_rot;
%               Kd                               0                      -Kd;
%               Cd/J_gen                       1/J_gen            -Cd/J_gen];
          
     LWT.A = [(gamma-Cd)/J_rot              -Kd/J_rot             Cd/J_rot     beta/J_rot      0;
              1                               0                      -1           0            0;
              Cd/J_gen                       Kd/J_gen            -Cd/J_gen        0            0;    
              0                                 0                   0             0            1;
              0                                 0                   0           -w^2       -2*xi*w];

      LWT.B  = [ 0   0   0   0  w^2]';
      LWT.B_d = [ alpha/J_rot 0  0  0  0 ]';
      
      LWT.Cy = [ 1 0 0 0 0;
                 0 0 1 0 0;
                 0 0 0 1 0];      
      LWT.C = [ 1 0 0 0 0];
      
      LWT.D = 0;
      LWT.Dy = 0;
      out = LWT;
      return
end




end




