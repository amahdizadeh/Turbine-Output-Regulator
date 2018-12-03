function [Parameter] = NREL5MWDefaultParameterFAST
%% SLOW
[Parameter] = NREL5MWDefaultParameterSLOW;

%% Modifications from SLOW
Parameter.PitchActuator.Mode            = 2;                % 0: none; 1: Delay; 2: PT2 (omega, xi, x1_con)

%% FAST, AeroDyn     
Parameter.FAST.SFuncName                = 'FAST_SFunc';
                                    
%% Steady Modes 
Parameter.CPC.SM                        = load('SteadyModesNREL5MW');
