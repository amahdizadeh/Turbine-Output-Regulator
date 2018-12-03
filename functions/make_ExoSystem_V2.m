function out = make_ExoSystem_V2(Signal)
%
% make_ExoSystem:  make an exo system, mimicing the given signal
%
%   out = make_ExoSystem(Signal,order)
%
%   Signal: a 4xN signal containing time values in first column,   
%           signal values & derivitives in the next columns
%   order : order of the differential equations to be estimated
%
%   Dependencies: 


%%  trimming the data
N=length(Signal);
sig_Data = Signal;
%sig_Data(:,2) = sig_Data(:,2) - sig_Data(end,2);        % make the final signal value to be Zero : Signal(end) = 0
sig_Data(:,1) = sig_Data(:,1) - sig_Data(1,1);          % make Time(1) = 0

%% search for coeeficients
vec_coef = estimate_GreyBox_Coeffs(Signal);

%% Simulate exo system
tspan = sig_Data(:,1);      % time span
x0 = sig_Data(1,2:end);     % initial condition
ode_options = odeset('RelTol',1e-4,'AbsTol',ones(length(vec_coef),1)* 1e-4);
[Time,y] = ode45(@ode_LinAutoSys,tspan,x0,ode_options,vec_coef);   

%% make ExoSystem
A = eye(length(vec_coef));A = circshift(A,-1);A(end,:)=vec_coef';
ExoSys.S = A;
ExoSys.L = [1 , zeros(1,2)];
ExoSys.X0 = sig_Data(1,2:end);

ExoSys.output = [Time,y(:,1)];
ExoSys.Error = sum(( y(:,1) - sig_Data(:,2)).^2)/N;
ExoSys.Offset = Signal(end,2);

out = ExoSys;


