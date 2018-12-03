function out = make_initial_states(wind_speed,Parameter)

%% make initial states given wind speed
 out =             [ Wind_Xt_Table(wind_speed);
                   0;
                   Parameter.CPC.Omega_rated;
                   Wind_Theta_Table(wind_speed) *pi/180;
                   0];

end