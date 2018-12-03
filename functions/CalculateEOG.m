function [t v]=CalculateEOG(t_EOG_Start,Parameter)

%% Defaults
DefaultValues   = {
    'Parameter.Time.TMin'       '0'
    };
for iDefaultValue=1:size(DefaultValues,1)
    try     eval([DefaultValues{iDefaultValue,1},';']);
    catch   eval([DefaultValues{iDefaultValue,1},'=',DefaultValues{iDefaultValue,2},';']);
    end
end
%% internal variables
URef            = Parameter.TurbSim.URef;
dt              = Parameter.Time.dt;
TMin            = Parameter.Time.TMin;
TMax            = Parameter.Time.TMax;
RotorDiameter   = Parameter.Turbine.RotorDiameter;
HubHeight       = Parameter.Turbine.HubHeight;


%% EOG for Class IA
v_ref           = 50;
I_ref           = 0.16;

sigma_1         = I_ref*(3/4*URef+5.6);

Lambda_1        =((0.7*HubHeight)*(HubHeight<60)+(42)*(HubHeight>=60));
v_e50           = 1.4*v_ref;
v_e1            = 0.8*v_e50;
v_gust          = min(1.35*(v_e1-URef),3.3*sigma_1/(1+0.1*RotorDiameter/Lambda_1));
T               = 10.5;
t_eog           = 0:dt:10.5;
v_eog           = URef - 0.37*v_gust*sin(3*pi*t_eog/T).*(1-cos(2*pi*t_eog/T));

%% interpolation
t               = [TMin:dt:TMax]';
v               = interp1(t_EOG_Start+t_eog,v_eog,t,'linear',URef);
