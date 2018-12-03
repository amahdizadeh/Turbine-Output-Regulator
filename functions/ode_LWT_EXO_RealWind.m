function dx = ode_LWT_EXO_RealWind(t,x,info)

persistent G;
%% linearized wind turbine + controller 
LWT = info.sysparam;
Exo1 = info.Exo1;
Exo2 = info.Exo2;
L1 = Exo1.L; 
L2 = Exo2.L;

n=length(LWT.A);        % order of the plant
p=length(L1);           % order of the disturbance Exo system
q=length(L2);           % order of the Reference Exo system

%% states of the system:
% States 1 to 5   :        xT, xT_dot, Omega, Theta, Theta_dot
% States 6 to 8   :        Disturbance exo system states
% States 9 to 12  :        Reference Exo-system; 9: xT_ref
% states 12 to end:        Scope signals
  dx = zeros(n+p+q+2,1);  

                   
Ew = [LWT.B_d*L1, zeros(5,length(L2))];

% disturbance = info.dist;
% R = info.setpoint;
F = info.controller;

% R = info.w 
% dx = (LWT.A+LWT.B*F)*x + LWT.B*R + LWT.B_d*disturbance;
% w = info.w ;
[Pi, Gamma] = Sylv_solver_Exo(LWT,Exo1,Exo2);
% Pi = sylv(1);
% Gamma = sylv(2);

if (isempty(G))
    G = Gamma - F * Pi;
end
%  G
% U = F*x + R;

Eta =  x(n+1:n+p);             % Disturbance Exo states
Zeta = x(n+p+1:n+p+q);             % Reference Exo states
X = x(1:n);                % system states

rt = (info.k - 1)*1.5 + t;

w_dt0_real = interp1( info.Wind_Real(:,1),  info.Wind_Real(:,2),rt*.99);
w_dt1_real = interp1( info.Wind_Real(:,1),  info.Wind_Real(:,3),rt*.99);
w_dt2_real = interp1( info.Wind_Real(:,1),  info.Wind_Real(:,4),rt*.99);

w_dt0_filt = interp1( info.Wind_Filt(:,1),  info.Wind_Filt(:,2),rt*.99);
w_dt1_filt = interp1( info.Wind_Filt(:,1),  info.Wind_Filt(:,3),rt*.99);
w_dt2_filt = interp1( info.Wind_Filt(:,1),  info.Wind_Filt(:,4),rt*.99);


W = [Eta ; Zeta];

% if (true)  
%     alpha = 0.2;
%     beta = 0.99;
%     %W = [w_t w_dt w_ddt Zeta(1) w_t w_dt w_ddt]';
%     %W(1) = (w_t + Eta(1) )/2; 
%     W(2) = (beta * w_dt + (1-beta)*Eta(2) );
%     W(3) = w_ddt;
%     
%     
%     W(5) = (w_t + Zeta(2) )/2;    
%     %W(5) = w_t;
%     W(6) = (alpha * w_dt + (1-alpha)*Zeta(3) );
%     W(7) = w_ddt;
% end

if (true)  
    W(1) = w_dt0_filt ;
    W(2) = w_dt1_filt;
    W(3) = w_dt2_filt;
    
    W(4) = w_dt0_filt;
    W(5) = w_dt1_filt;
    W(6) = w_dt2_filt;

    
    W(7) = w_dt0_filt;
    W(8) = w_dt1_filt;
    W(9) = w_dt2_filt;
else
    W(1) = w_dt0_real ;
    W(2) = w_dt1_real;
    W(3) = w_dt2_real;

    W(5) = w_dt0_real;
    W(6) = w_dt1_real;
    W(7) = w_dt2_real;
    
end


%t
% j = [ rt , Eta(1) - w_t]
U = F*X + G*W;

% differential equations
% dx(1:5) = LWT.A*X + LWT.B*U + LWT.B_d*disturbance;      % system states
% dx(1:n) = LWT.A*X + LWT.B*U + Ew*W;      % system states (formulation in paper)

wind =  interp1( Exo1.output(:,1),  Exo1.output(:,2),t*.99);
wind =  interp1( Exo1.RealData(:,1),  Exo1.RealData(:,2),t*.99);
dx(1:n) = LWT.A*X + LWT.B*U + LWT.B_d*wind;      % system states ( general formulation)

dx(n+1:n+p) = info.Exo1.S*Eta;            % dist exo system states
dx(n+p+1:n+p+q) = info.Exo2.S*Zeta;               % ref exo system states 

% measuring section
prob_Gain = 100;
 dx(n+p+q+1:n+p+q+2)=-prob_Gain*eye(2)*x(n+p+q+1:n+p+q+2) + ( U).*prob_Gain;
%dx(n+p+q+1:n+p+q+2)=-prob_Gain*eye(2)*x(n+p+q+1:n+p+q+2) + [eig(G*G')].*prob_Gain;
