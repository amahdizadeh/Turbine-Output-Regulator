
load('empirical_eog9d2-2.mat')

subplot(511)
plot(Disturbance.v_0.time,Disturbance.v_0.signals.values)
hold on;grid on; box on;
ylabel('v_0 [m/s]')

subplot(512)
plot(tout,rad2deg(Results.X(4,:)))
hold on;grid on; box on;
ylabel('\theta [deg]')
% xlim([0 Parameter.Time.TMax])

subplot(513)
hold on;grid on; box on;
plot(tout,Results.X(6,:))
ylabel('M_g [Nm]')
% xlim([0 Parameter.Time.TMax])

subplot(514)
hold on;grid on; box on;
plot(tout,radPs2rpm(Results.X(1,:)))
ylabel('\Omega [rpm]')
% xlim([0 Parameter.Time.TMax])

subplot(515)
hold on;grid on; box on;
plot(tout,1e-6* M_yT_NMPC )
ylabel('M_{yT} [MNm]')



