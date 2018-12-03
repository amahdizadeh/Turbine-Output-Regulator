function dx = ode_Full_WT(t,x,Controller,Parameter,Disturbance)
%% Full nonlinear model of the turbine + controller

% disturbance = info.dist;
% R = info.setpoint;
% F = info.controller;


% local variables ---------------------------------------------------------
TBN         = Parameter.Turbine;

v_wind      = interp1(Disturbance.v_0.time,...    % find the current wind speed       
                      Disturbance.v_0.signals.values,...
                      t);





x_T         = x(1);          	% Tower displacement 
x_T_dot     = x(2);            	% Tower displacement speed
Omega       = x(3);          	% Hub's rotational speed 
theta       = x(4);           	% Pitch Angle
theta_dot   = x(5);           	% Pitch Angle rate
M_g         = 43635;            % generator torque

Mg_rated    = 43635;
% -------------------------------------------------------------------------


%% controller action
theta_c     = 0.1824;
% theta_c     = theta - (Parameter.CPC.Omega_rated - Omega)*3;
work_point = Controller.work_point;

Wind_Points = [11.4, 12:25];                            % linearization points

i = find(Wind_Points == floor(v_wind)); 
A = Controller.LWT(i).A;
B = Controller.LWT(i).B;
C = Controller.LWT(i).C;
B_d = Controller.LWT(i).B_d;

Wind_base = floor(v_wind);
work_point = make_initial_states(Wind_base,Parameter);
% F = Controller.FBK_table_place(:,:,i);
nous_act =0;
if (nous_act == 0)
    F = -place(A,B,Controller.NOUS.cmode_U(Wind_base,:));
    F = -place(A,B,[-.3 -.12 -.4 -.2 -.8]);
else
    F = Controller.NOUS.F_U(:,:,Wind_base);
end
Ac = A +B*F;
G = C/Ac*B;
G_d = C/Ac*B_d;

delta_Xt =  Wind_Xt_Table(v_wind) - Wind_Xt_Table(Controller.wind_speed);
delta_d = v_wind - Wind_base;
delta_Yss =   [delta_Xt  0 ]';
delta_Uss= -G\delta_Yss - G\G_d*delta_d;


delta_x =  x - work_point;
delta_U = delta_Uss  + F * delta_x ; 

% U =  Controller.Update_ut(v_wind,x);
% delta_U(1)
[delta_U(1) t];
M_g = Mg_rated + delta_U(1);
theta_c = Wind_Theta_Table(ceil(v_wind))/180*pi + delta_U(2); 

%% Aerodynamics
M_a         = CalculateAerodynamicTorque(x_T_dot,Omega,theta,v_wind,Parameter);
F_a         = CalculateAerodynamicThrust(x_T_dot,Omega,theta,v_wind,Parameter);

%% Equations
dx          = zeros(5,1);    % a column vector
%--
dx(1)       = x_T_dot;
dx(2)       = 1/TBN.m_Te    * ( F_a - TBN.c_Te*x_T_dot - TBN.k_Te*x_T);
dx(3)       = 1/TBN.J       * ( M_a - M_g/TBN.i);
dx(4)       = theta_dot;
dx(5)       = - 2* TBN.zeta * TBN.w * theta_dot - (theta - theta_c) * TBN.w^2 ;


end
