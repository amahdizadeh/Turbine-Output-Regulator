function            [Parameter]     = NREL5MWSecondaryParameterSLOW(Parameter,Disturbance)
%% FF
Parameter.CPC.FF.T_buffer       = Parameter.Scan.T_preview-Parameter.CPC.FF.tau;

%% IC                       
Parameter.IC.v_0                = Disturbance.v_0.signals.values    (sum((Disturbance.v_0.time  <=0))); % value nearest to t=0;
Parameter.IC.v_0L               = Disturbance.v_0L.signals.values   (sum((Disturbance.v_0L.time	<=0))); % value nearest to t=0;
Parameter.IC.theta              = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.theta   ,Parameter.IC.v_0,'linear','extrap');
Parameter.IC.M_g                = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.M_g     ,Parameter.IC.v_0,'linear','extrap');
Parameter.IC.Omega              = max(interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.Omega   ,Parameter.IC.v_0,'linear','extrap'),0);
Parameter.IC.x_T                = interp1(Parameter.CPC.SS.v_0,Parameter.CPC.SS.x_T     ,Parameter.IC.v_0,'linear','extrap');
Parameter.IC.Omega_g            = Parameter.IC.Omega/Parameter.Turbine.i ;

end