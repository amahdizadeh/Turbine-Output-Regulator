function out = make_ExoSystem(Signal,SysOrder,CostCoeff,X0)
%
% make_ExoSystem:  make an exo system, mimicing the given signal
%
%   out = make_ExoSystem(Signal,order)
%
%   Signal: a 2xN signal containing time values in first column and signal
%           values in second column
%   order : order of the differential equations to be estimated
%
%   Dependencies: fitting_error, ode_LinAutoSys, ode_LinAutoSys, smoothed_diffs


%%  trimming the data
N=length(Signal);
sig_Data = Signal;
% sig_Data(:,2) = sig_Data(:,2) - sig_Data(end,2);        % make the final signal value to be Zero : Signal(end) = 0
sig_Data(:,1) = sig_Data(:,1) - sig_Data(1,1);          % make Time(1) = 0

%% search for coeeficients
TermCostCoef = CostCoeff;
options = optimoptions(@fminunc,'Algorithm','quasi-newton','TolFun',1e-4,'MaxFunEvals',1000);
if (nargin==3)
    [vec_coef,fval,exitflag,output]  = fminunc(@(p)fitting_error(p, sig_Data,TermCostCoef), -10*ones(SysOrder,1) ,options )
else
    [vec_coef,fval,exitflag,output]  = fminunc(@(p)fitting_error(p, sig_Data,TermCostCoef,X0), -10*ones(SysOrder,1) ,options )
end

%% Building  the exo system

initial_states = smoothed_diffs (sig_Data,length(vec_coef)-1,fix(N/10)+1,1);

if (nargin==4)
   Len_init = length(initial_states);
   Len_Fx0  = size(X0,2); 
   for i=1:min(Len_init,Len_Fx0)
       if (X0(1,i)~=0)
           initial_states(i) = X0(2,i);
       end
   end
    [ERR output]  = fitting_error(vec_coef,sig_Data,TermCostCoef,[X0(1,:) zeros(1,Len_init-Len_Fx0); initial_states]);
else
    [ERR output]  = fitting_error(vec_coef,sig_Data,TermCostCoef);
end
A = eye(length(vec_coef));A = circshift(A,-1);A(end,:)=vec_coef';
ExoSys.S = A;
ExoSys.L = [1 , zeros(1,SysOrder-1)];
ExoSys.X0 = initial_states';
ExoSys.output = output;
ExoSys.Error = ERR;
ExoSys.Offset = Signal(end,2);
ExoSys.RealData = sig_Data;

out = ExoSys;


