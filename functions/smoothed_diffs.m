function dx = smoothed_diffs (Signal,order,N_frame,Tn)
%
% Smoothed_Diffs:  calculating 0 to n'th differentiation of the given signal
%                  smoothing is carried out based on  Savitzky-Golay FIR smoothing filter 
%
%   dx = Smoothed_Diffs (Signal,order,Nth_smooth,Tn)
%
%   Signal     : a N*2 signal matrix containing time values in first column and 
%               signal values in second column
%   order      : maximum order of the differentiation 
%   N_frame    : smoothing frame length
%   Tn         : differentiation point


%%  scaling and trimming data
t = imresize(Signal(:,1),[500 1]);
s = imresize(Signal(:,2),[500 1]);
 t=t-t(1);              % trim to zero
 dt = mean(diff(t));
 %%

diffs = zeros(order+1,1);
diffs(1) = s(Tn);                           % value of the original signal at Tn
s_f = sgolayfilt(s,order+1,N_frame);        % smoothing the signal
for i=1:order
    tmp = diff(s_f,i)/dt^i;
    diffs(i+1) = tmp(Tn);
end


dx = diffs';