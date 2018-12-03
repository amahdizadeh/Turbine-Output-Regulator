function out = Append_SimulinkSignals (Signal_1, Signal_2,Timegap)
%% Appends to simulink signals together Signal = Signal_1 -> Timegap -> Signal_2
    
    %% reading
    time1 = Signal_1.time';
    time2 = Signal_2.time';
    val1 = Signal_1.signals.values;
    val2 = Signal_2.signals.values;
    %% appending
    time2 = time2 + time1(end) + Timegap;
    sig=[];
    sig.time = [time1 ; time2];
    sig.signals.values = [val1 ; val2];
    sig.dimensions = size(val1,2);
    %% return
    out = sig;    
end
