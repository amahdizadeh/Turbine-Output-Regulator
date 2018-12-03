%% MWE_CalculationDEL
% Script: This is a Minimum Working Example to Calclate DEL using the RFC
% of Nieslony. The data are the same as in [1], see Table 6.3.
% [1] D. Schlipf. \Lidar-Assisted Control Concepts for Wind Turbines." 
% PhD thesis, University of Stuttgart,2016, DOI: 10.18419/opus-8796.
% 
%% Created: 
% David Schlipf on 18-May-2016

% clear all;clc;close all?
%addpath('rfcNieslony')

%% load Data
% load('NLStates.mat');
Rec = Measurments.Data;

dt              = mean(diff(Measurments.Time));
START = TestProfile.DEL_Start;
END = Parameter.Time.TMax;
LEN = END - START;

Time = (0:dt:(END-START))'; 
Data = Rec(fix(START/dt):fix(END/dt),9);
%% configuration
WoehlerExponent =  4; 
N_REF           = 2e6; 
SECINLIFETIME   = 20*8760*3600; 
% dt              = diff(Time(1:2)); 
TimeOfBlocks    = Time(end)-Time(1);

%% calculation of DEL
DEL.MyT.Value            = ComputeDamageEquivalentLoad(Data,dt,N_REF*TimeOfBlocks/SECINLIFETIME,WoehlerExponent);
DEL.MyT.Unit             = 'K.Nm';

DEL.MyT.Value