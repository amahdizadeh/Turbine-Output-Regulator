% generating EOG wind for transition region


clear all;
clc


% define input parameters
addpath('Tools');
Parameter = NREL5MWDefaultParameterFAST;        %calling DEFAULT parameters

Parameter.Time.TMax = 300;                      
Parameter.Time.dt =0.00625; %0.0125;
Parameter.TurbSim.URef = 20;

%outputs 
[t, v] = CalculateEOG(50 , Parameter);
plot(t, v)

%write the wind for NMPC
Wnd = v;
%HHData =[t, v];
save 'EOG_20_50_dt625'


