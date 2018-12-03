% Function: Quick 2D-Interpolation.
% based on interp2 from FloatingWindTurbine_CP.cpp
% see http://en.wikipedia.org/wiki/Bilinear_interpolation
% 
%% Usage:
% 
% ZI = QuickInterp2(X,Y,Z,XI,YI)
% 
%% Input:
% 
%
% 
%% Output:
% 
% 
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
% David Schlipf on 07-Aug-2015
%
% (c) Universitaet Stuttgart
% 
%% Code
function ZI = QuickInterp2(X,Y,Z,XI,YI)
%#codegen   


% Find X and Y values
nX          = length(X);
nY          = length(Y);
[~,IndexX]  = histc(XI,X);
[~,IndexY]  = histc(YI,Y);
IndexX      = max(IndexX,1);
IndexX      = min(IndexX,nX-1);
IndexY      = max(IndexY,1);
IndexY      = min(IndexY,nY-1);
X1          = X(IndexX  );
X2          = X(IndexX+1);
Y1          = Y(IndexY  );
Y2          = Y(IndexY+1);

% Z values
Z11         = Z(IndexY  ,IndexX  );
Z12         = Z(IndexY  ,IndexX+1);
Z21         = Z(IndexY+1,IndexX  );
Z22         = Z(IndexY+1,IndexX+1);

% Interpolation        
ZI          =   (Z11*(X2-XI)*(Y2-YI)...
                +Z12*(XI-X1)*(Y2-YI)...
                +Z21*(X2-XI)*(YI-Y1)...
                +Z22*(XI-X1)*(YI-Y1))/(X2-X1)/(Y2-Y1);
end
        
