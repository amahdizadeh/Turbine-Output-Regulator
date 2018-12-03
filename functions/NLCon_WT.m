function [c , ceq ] = NLCon_WT(s,x_0,u_0,t_s,d_s,Parameter)
% local variables ---------------------------------------------------------
dt          = Parameter.NMPC.dt;
N           = Parameter.NMPC.N;
% -------------------------------------------------------------------------
switch Parameter.NMPC.NONLCON.Mode
    case 'Omega_max-theta_min-theta_max'
        
        % Update Trajectory
        U(1,:)      = [u_0(1) s(1:N)'    *Parameter.NMPC.u_cond(1)];
        U(2,:)      = [u_0(2) s(N+1:2*N)'*Parameter.NMPC.u_cond(2)];
        
        % Simulation
        nMinorSteps = dt/Parameter.Time.dt;
        M           = repmat(2:N+1,nMinorSteps,1);
        Idx         = [1;M(:)];
        
        u_s     	= U(:,Idx);
        x_s        	= WindTurbine_ODE_DS(x_0,t_s,u_s,d_s,Parameter);
        X           = x_s(:,1:nMinorSteps:N*nMinorSteps+1);
        
        
        %----------------------------------------------------------------------------------------
        % inequality constraints  c(x) <= 0
        c1 = ( +X(1,:)-Parameter.NMPC.NONLCON.Omega_max);       %  Omega < 1.2 * Omega_rated
        c2 = ( -X(5,:)+Parameter.NMPC.NONLCON.theta_dot_min);  	%  -8<= x_5 
        c3 = ( +X(5,:)-Parameter.NMPC.NONLCON.theta_dot_max); 	%       x_5 <= +8
%         c4 = ( +X(4,:)-pi/2);
%         c5 = ( -X(4,:));
        
        c  = [c1; c2;c3];
%         c  = [c1; c2;c3; c4; c5];
        ceq=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    case 'none'
        c   = [];
        ceq = [];
end


end