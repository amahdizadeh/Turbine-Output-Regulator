% Function radPs2rpm: converts from rad per second to rounds per minute
%
% -----------------------------
% Usage:
% -------------
%  y = radPs2rpm(u)
% ------------
% Input:
% -------------
% - u
% ------------
% Output:
% -------------
% - y
%------------
% Needs:
%-------------
%       -
%-----------
% Modified:
%-------------
%
%-----------
% ToDo:
%-------------
% 
% -------------
% Created:
% David Schlipf on          2009-01-11
% (c) Universitaet Stuttgart
% ----------------------------------

function y = radPs2rpm(u)
y = u * 60/(2*pi);
