%% generating EOG wind profile 
clear all;
clc
addpath('Tools');

% define input parameters
Parameter = NREL5MWDefaultParameterFAST;       
Parameter.Time.TMax = 200;                      
Parameter.Time.dt =0.0125;
Parameter.TurbSim.URef = 20;

% calling EOG function 
[t, v] = CalculateEOG(10, Parameter);

 plot(t, v)
 save('EOG_20_30.mat','t','v')

%% create *.wnd wind file for FAST
mat2wnd([t, v],'EOG20_30.wnd')

%%