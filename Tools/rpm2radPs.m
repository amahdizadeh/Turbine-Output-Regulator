%% rpm2radPs
% Function : converts from rounds per minute to rad per second
%
%% Usage:
% 
% y = rpm2radPs(u)
% 
%% Input:
% 
% [u] = rounds per minute
% 
%% Output:
% 
% [y] = rad per second
%
%% Modified:
%
%
%
%% ToDo:
%
% 
% 
%% Created:
% David Schlipf on          2009-01-11
%
% (c) Universitaet Stuttgart
% 

function y = rpm2radPs(u)
y = u * 2*pi/60;
