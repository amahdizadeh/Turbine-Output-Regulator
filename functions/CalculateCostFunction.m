function [fval,F,F_M] = CalculateCostFunction(X,U,D,Parameter)
% local variables ---------------------------------------------------------
Q           = Parameter.NMPC.Q;
R           = Parameter.NMPC.R;
% -------------------------------------------------------------------------
switch Parameter.NMPC.COSTFN.Mode
    
    %%% region 3 (elham)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    case 'Omega-P_el-x_T_dot-Theta_dot-M_g_dot-theta_c'
        % calculation cost function
        P_el        = X(1,:).*X(6,:)*Parameter.Turbine.eta/Parameter.Turbine.i;

        F_M(1,:)    =   Q(1)*(X(1,:)-Parameter.CPC.Omega_rated).^2  * 1/Parameter.CPC.Omega_rated^2  ;
        F_M(2,:)    =   Q(2)*(P_el  -Parameter.VSC.P_el_rated ).^2  * 1/Parameter.VSC.P_el_rated^2   ;
        F_M(3,:)    =   Q(3)*(X(3,:)).^2    ;                       % divide by the maximum tower natural frequency? !!        
        F_M(4,:)    =   Q(4)*(X(5,:)).^2    ;                       % divide by the maximum tower natural frequency? !!        
        F_M(5,:)    =   R(1)*(U(1,:)).^2                            * 1/Parameter.VSC.M_g_dot_max^2 ;
        F_M(6,:)    =   R(2)*(U(2,:)).^2                            * 1/Parameter.CPC.theta_dot_max^2;            


        F           = sum(F_M);                                           
        fval        = sum(F); 
               
%%% region 2/transition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        case 'Omega-Mg-x_T_dot-theta_dot_M_g_dot-theta_c'
           
%---------------% switching between region 2 and 3)

         % Auxillary equations
        P_el        = X(1,:).*X(6,:)*Parameter.Turbine.eta/Parameter.Turbine.i;      
        nSim        = length(D);
        
        v_rated     = Parameter.CPC.wind_rated;
        v_2to2d5    = Parameter.Turbine.R .* Parameter.CPC.Omega_rated / Parameter.VSC.lambda_opt;     % a specific wind speed
        
        % calculating Omega_ref
        Omega_opt   = D.* Parameter.VSC.lambda_opt / Parameter.Turbine.R;
        Omega_ref   = min(Omega_opt,Parameter.CPC.Omega_rated); 

        % calculating M_g_ref 
        M_g_ref   = zeros(1, nSim); % allocation
        
        for iSim = 1:nSim
            v  = D(iSim);
            if      v < v_2to2d5                    % R2
                Mg   = Parameter.VSC.k *(Omega_opt(iSim)/Parameter.Turbine.i)^2;
            elseif  v < v_rated                     % R2.5
                M_a             = CalculateAerodynamicTorque(0,Parameter.CPC.Omega_rated,0,v,Parameter);
                Mg   = M_a *Parameter.Turbine.i;
            else
                Mg   = Parameter.VSC.M_g_rated;     % R3
            end
            M_g_ref(iSim) = Mg;
        end

     M_g_ref ;         
%----------------------% calculation cost function-------------------------
        
        F_M(1,:)    =   Q(1)*(X(1,:)- Omega_ref ).^2  ; %* 1/Parameter.CPC.Omega_rated^2  ;
        F_M(2,:)    =   Q(2)*(X(6,:)- M_g_ref   ).^2  * 1/Parameter.VSC.M_g_rated^2    ;
        F_M(3,:)    =   Q(3)*(X(3,:)).^2    ;          % divide by the maximum tower natural frequency? !!        
        F_M(4,:)    =   Q(4)*(X(5,:)).^2             * 1/Parameter.CPC.theta_dot_max^2;            
        F_M(5,:)    =   R(1)*(U(1,:)).^2             * 1/Parameter.VSC.M_g_dot_max^2 ;    % Mg_dot
        F_M(6,:)    =   R(2)*(U(2,:)).^2             * 1/Parameter.CPC.theta_max^2;       % Theta_c       



        F           = sum(F_M);                                           
        fval        = sum(F);
end

end
%%
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
