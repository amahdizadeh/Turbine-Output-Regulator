function PlotNMPCResults(tout,Results,Disturbance,M_yT_NMPC,M_yT_BL,Parameter)
% DS on 30-Jul-2015

% %M_yT= 90 *( 1.7782e4 *x_dot_T + 1.81e6 * x_T)
%  A= (Results.X_BL(2,:)) ; 
%  B= diff(A);
%  C=[0;B];
%  
%  M_yT_BL = 90 *( 1.81e6  * A + 1.7782e4 *C);
% 
%  M_yT_NMPC = 90 *( 1.81e6  * (Results.X(2,:)) + 1.7782e4 *(Results.X(3,:)));

figure('Name','Simulation Results')

subplot(611)
hold on;grid on; box on;
plot(Disturbance.v_0.time,Disturbance.v_0.signals.values)
ylabel('v_0 [m/s]')
% xlim([0 Parameter.Time.TMax])

subplot(612)
hold on;grid on; box on;
% plot(tout,rad2deg(Results.X(4,:)))
plot(tout,rad2deg(Results.U(2,:)) ,'g')
plot(tout,rad2deg(Results.U_BL(2,:)),'r')
ylabel('\theta_c [deg]')
% xlim([0 Parameter.Time.TMax])

subplot(613)
hold on;grid on; box on;
plot(tout,Results.X(6,:))
plot(tout,Results.U_BL(1,:),'r')
ylabel('M_g [Nm]')
% xlim([0 Parameter.Time.TMax])

subplot(614)
hold on;grid on; box on;
plot(tout,radPs2rpm(Results.X(1,:)))
plot(tout,radPs2rpm(Results.X_BL(1,:)),'r')
ylabel('\Omega [rpm]')
% xlim([0 Parameter.Time.TMax])

subplot(615)
hold on;grid on; box on;
plot(tout,(Results.X(2,:)))
plot(tout,(Results.X_BL(2,:)),'r')
ylabel('x_T [m]')
% xlim([0 Parameter.Time.TMax])

% subplot(616)
% hold on;grid on; box on;
% plot(tout,1e-6*(Results.X(1,:).*Results.X(6,:)*0.944*97) )
% plot(tout,1e-6*(Results.X_BL(1,:).*Results.U_BL(1,:)*0.944*97),'r')
% ylabel('P_el [MW]')
%  xlim([0 Parameter.Time.TMax])

%figure;
subplot(616)
hold on;grid on; box on;
plot(tout,1e-6* M_yT_NMPC )
plot(tout,1e-6*M_yT_BL ,'r')
ylabel('M_{yT} [MNm]')
% xlim([0 Parameter.Time.TMax])

figure
hold on;grid on; box on;
plot(tout,(Results.U(1,:)) ,'g')
plot(tout,rad2deg(Results.U_BL(2,:)),'r')
end

