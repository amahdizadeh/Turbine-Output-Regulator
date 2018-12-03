function            [Parameter]     = NREL5MWSecondaryParameterFAST(Parameter,Disturbance)
%% SLOW
[Parameter]     = NREL5MWSecondaryParameterSLOW(Parameter,Disturbance);

%% IC   
NDOF                            = 24;

for iDOF=1:NDOF
    Parameter.IC.q(iDOF)    	= interp1(Parameter.CPC.SM.v_0,Parameter.CPC.SM.q(iDOF,:),Parameter.IC.v_0,'linear','extrap');
end

Parameter.IC.qdot            	= zeros(1,NDOF);
Parameter.IC.qdot(13)           = Parameter.IC.Omega;
end