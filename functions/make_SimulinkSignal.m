function out = make_SimulinkSignal (Time,Signal)
%% Converts a vector of time and a vector of values to a simulink usable signal source

    if (size(Time,1) ~=size(Time,1))
        error('Time and Signal must have a same lentgh')
        out = [];
    end
    sig = [];
    sig.time = Time;
    sig.signals.values = Signal;
    sig.dimensions = size(Signal,2);
    sig.label = 'GxW';
    %%
    out = sig;
    
end

% 
% states = 
%          time: [3201x1 double]
%       signals: [1x1 struct]
%     blockName: 'NREL5MW_EOR_FAST_R3/FAST/To Workspace'
% states.signals
% ans = 
%         values: [3201x24 double]
%     dimensions: 24
%          label: 'q'