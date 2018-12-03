function PlotNMPCTrajectory(X,U,t_v,v_0_v,t_0,Parameter)
% DS on 30-Jul-2015
% local variables ---------------------------------------------------------
dt              = Parameter.NMPC.dt;
N               = Parameter.NMPC.N;
MyColorMatrix   = Parameter.NMPC.MyColorMatrix;
nPreview     	= Parameter.NMPC.nPreview;
% -------------------------------------------------------------------------

t_horizon       = t_0+[0:N]*dt;
D               = interp1(t_v,v_0_v,t_horizon);

[~,~,F_M]       = CalculateCostFunction(X,U,D,Parameter);

if t_0<dt % init
    figure('Name','Preview')
    
    subplot(611)
    hold on;grid on; box on;
    ylabel('v_0 [m/s]')
    set(gca,'xTicklabel',[]);

    subplot(612)
    hold on;grid on; box on;
    ylabel('{\theta}_c [deg]')
    set(gca,'xTicklabel',[]);

    subplot(613)
    hold on;grid on; box on;
    ylabel('M_g [Nm]')
    set(gca,'xTicklabel',[]);
    
    subplot(614)
    hold on;grid on; box on;
    ylabel('x_T [m]')
    set(gca,'xTicklabel',[]);    

    subplot(615)
    hold on;grid on; box on;
    ylabel('\Omega [rpm]')
    set(gca,'xTicklabel',[]);     
    
    subplot(616)    
    ylabel('F [-]')
    xlabel('time [s]')    
end

x               = t_horizon;
subplot(611)    
y               = D;
plot(x,y,'.-','Tag','Preview001','Color',MyColorMatrix(1,:))
xlim([x(1),x(end)])
set(gca,'xTick',t_horizon)
ylimDeltaMin    = 1.0;
ylimMean        = round(y/ylimDeltaMin)*ylimDeltaMin;
ylimMin         = min([y,ylimMean-ylimDeltaMin]);
ylimMax         = max([y,ylimMean+ylimDeltaMin]);
ylim([ylimMin,ylimMax])

subplot(612)
y               = rad2deg(U(2,:));
plot(x,y,'.-','Tag','Preview001','Color',MyColorMatrix(1,:))
xlim([x(1),x(end)])
set(gca,'xTick',t_horizon)
ylimDeltaMin    = 1.0;
ylimMean        = round(y/ylimDeltaMin)*ylimDeltaMin;
ylimMin         = min([y,ylimMean-ylimDeltaMin]);
ylimMax         = max([y,ylimMean+ylimDeltaMin]);
ylim([ylimMin,ylimMax])

subplot(613)
y               = X(6,:);
plot(x,y,'.-','Tag','Preview001','Color',MyColorMatrix(1,:))
xlim([x(1),x(end)])
set(gca,'xTick',t_horizon)
ylimDeltaMin    = 10.0;
ylimMean        = round(y/ylimDeltaMin)*ylimDeltaMin;
ylimMin         = min([y,ylimMean-ylimDeltaMin]);
ylimMax         = max([y,ylimMean+ylimDeltaMin]);
ylim([ylimMin,ylimMax])

subplot(614)
y               = X(2,:);
plot(x,y,'.-','Tag','Preview001','Color',MyColorMatrix(1,:))
xlim([x(1),x(end)])
set(gca,'xTick',t_horizon)
ylimDeltaMin    = 0.1;
ylimMean        = round(y/ylimDeltaMin)*ylimDeltaMin;
ylimMin         = min([y,ylimMean-ylimDeltaMin]);
ylimMax         = max([y,ylimMean+ylimDeltaMin]);
ylim([ylimMin,ylimMax])    

subplot(615)
y               = radPs2rpm(X(1,:));
plot(x,y,'.-','Tag','Preview001','Color',MyColorMatrix(1,:))
xlim([x(1),x(end)])
set(gca,'xTick',t_horizon)
ylimDeltaMin    = 0.1;
ylimMean        = round(y/ylimDeltaMin)*ylimDeltaMin;
ylimMin         = min([y,ylimMean-ylimDeltaMin]);
ylimMax         = max([y,ylimMean+ylimDeltaMin]);
ylim([ylimMin,ylimMax])

subplot(616)
y               = F_M;
[n_F,~]         = size(F_M);
plot(x,y,'.-')
grid on; box on;    
xlim([x(1),x(end)])
set(gca,'xTick',t_horizon)
legend(num2str([1:n_F]'),'Location','North','Orientation','horizontal')


%% Tags
drawnow
for iPreview=nPreview:-1:1
    set(findobj('Tag',['Preview',num2str(iPreview,'%03.0f')]),'Tag',['Preview',num2str(iPreview+1,'%03.0f')],'Color',MyColorMatrix(iPreview,:));
end
delete(findobj('Tag',['Preview',num2str(nPreview+1,'%03.0f')]));
