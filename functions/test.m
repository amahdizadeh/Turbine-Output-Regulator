clear 
clc

t0      = 0.0;                                      %simulation start time
tf      = 100;                                       %simulation period

Tc      = 0.2 ;                                     % control horizon (interval-time step)
Tp      = 5.6  ;                                    % prediction horizon

tk = [t0:Tc:tf];       

load('Wind_20_60.mat');                  % perfect wind measurement
 v_0 = [t, v]; 

wind = [tk', interp1(v_0(:,1), v_0(:,2), tk')];