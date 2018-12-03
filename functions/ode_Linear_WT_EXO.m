function dx = ode_LWT_EXO_SAT(t,x,info)
%% linearized wind turbine + controller 
LWT = info.sysparam;
Exo1 = info.Exo1;
Exo2 = info.Exo2;
L1 = Exo1.L; 
L2 = Exo2.L;

n=length(LWT.A);        % order of the plant
p=length(L1);           % order of the disturbance Exo system
q=length(L2);           % order of the Reference Exo system


dx = zeros(n+p+q,1);


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
G = Gamma - F * Pi;
% U = F*x + R;

Eta =  x(n+1:n+p);             % Disturbance Exo states
Zeta = x(n+p+1:n+p+q);             % Reference Exo states
X = x(1:n);                % system states


W = [Eta ; Zeta];
U = F*X + G*W;

% differential equations
% dx(1:5) = LWT.A*X + LWT.B*U + LWT.B_d*disturbance;      % system states
dx(1:n) = LWT.A*X + LWT.B*U + Ew*W;      % system states (formulation in paper)

% wind =  interp1( Exo1.output(:,1),  Exo1.output(:,2),t*.99);
% dx(1:n) = LWT.A*X + LWT.B*U + LWT.B_d*wind;      % system states ( general formulation)

dx(n+1:n+p) = info.Exo1.S*Eta;            % dist exo system states
dx(n+p+1:n+p+q) = info.Exo2.S*Zeta;               % ref exo system states    
