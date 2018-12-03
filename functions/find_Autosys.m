function out = find_Autosys(Signal,initState_unknown, retainative) %,SysOrder,init_Coeffs,init_States)
%
% find_Autosys:  finds an autonomous system as x_dot = Ax which fits the given signal
%
%   Signal            : a NxM signal containing time values in first column, 
%   second column to end contains signal value and its derivitives
%
%   Ref_Value         : assumed equilibrium point of the x_dot = Ax
%   initState_unknown : =0, when Signal is ued for initial states. =1, when inital 
%   states to be calculated as optimization variables too.  
%   retainative       : use previous round results as the next initial guess
%   Dependencies: calc_error, ode_LinAutoSys
%
% Written by: Amin Mahdizadeh, 10.08.2016

persistent init_guess;
     
     [~, M]=size(Signal);
    
    %% allocating search space dimenation (if initial condition to be searched or not)
    if (initState_unknown == 0)
        search_space_dim = M-1;
    else
        search_space_dim = (M-1)*2;
    end
    
    %% if retainative use is active, use previous local minima parameters as current initial guess
    if (retainative == 0)
        init_guess  = randn(1,search_space_dim);
    else
        if isempty(init_guess)
           init_guess  = randn(1,search_space_dim);
        end        
    end
    
    %% if Auto_ref = true, then find the reference automatically
%     if (Auto_ref == 0)
%         reference = Ref_Value;
        reference = Signal(end,2);
%     else      
%         reference = Signal(end,2);
%     end
    

    %% reading and triming the raw data
    sig_Data = Signal(1:end-1,:);                           % eliminate last data row ()
    sig_Data(:,1) = sig_Data(:,1) - sig_Data(1,1);          % make Time(1) = 0
    sig_Data(:,2) = sig_Data(:,2) - reference;              % make equilibrum value as reference
    sig_Data = imresize(sig_Data,[20 4]);

    %% performing the search via fminunc function
    tic
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','TolFun',1e-4,'MaxFunEvals',1000);
    % find a vector P which minimizes the calc_error function, size of 'P' = size of 'init_Param_Guess'
    [vec_coef,fval,exitflag,output] = fminunc(@(P)calc_error(sig_Data,P,initState_unknown),init_guess,options);
    toc
    
    init_guess = vec_coef;          % saving the data for next iteration
    
    %% shaping the output variable 'out'
    % first row : found system parameters,   
    % second row: initial condition (calculated or given)
    if (initState_unknown == 0)
        out = [vec_coef;  sig_Data(1,2:end)];
    else
        out = reshape(vec_coef,3,2)' ;
    end
end



function [err, out] = calc_error(sig,params,initState_unknown)

N = length(sig);
tf = sig(end,1);
tspan = linspace(0,tf,N);

param_len = length(params);

% calculating x(0) 
switch initState_unknown
    case 0
        sys_params = params;
        x0 = sig(1,2:4);
    case 1
        sys_params = params(1:param_len/2);
        x0 = params(param_len/2+1:end);
end


ode_options = odeset('RelTol',1e-4,'AbsTol',ones(length(sys_params),1)* 1e-4);
[Time,y] = ode45(@ode_LinAutoSys,tspan,x0,ode_options,sys_params);   

% returning the discrepency
err = sum(( y(:,1) - sig(:,2)).^2)/N ;
out = [Time, y(:,1)];
end
