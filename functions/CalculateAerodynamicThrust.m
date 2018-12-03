function F_a = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter,Coeffs)
    % local variables ---------------------------------------------------------
    TBN     = Parameter.Turbine;
    % -------------------------------------------------------------------------
    v_rel   = v_wind - x_T_dot;   % relative speed of tower and wind
    lambda  = Omega * TBN.R / v_rel;

    if (nargin < 6)
        C_T = CalculateTrustCoefficient(lambda,theta);
    else
        C_T =  interp2(Coeffs.Theta,Coeffs.Lambda,Coeffs.C_T,theta,lambda,'spline',0);
    end
    F_a     = 1/2 * TBN.rho * pi * TBN.R^2 *C_T * v_rel^2;
end