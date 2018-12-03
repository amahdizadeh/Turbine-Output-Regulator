% DS on 30-Jul-2015
clc
clear 
load('empirial_MPC2015_1.mat')
% load('PSO_2.mat')    %ok


figure('Name','Simulation Results')

subplot(611)
hold on;grid on; box on;
plot(Disturbance.v_0.time,Disturbance.v_0.signals.values, 'b', 'LineWidth',2.5 )
ylabel('v_0 [m/s]')
xlim([0 Parameter.Time.TMax])

subplot(612)
hold on;grid on; box on;
plot(tout,rad2deg(Results.X(4,:)), '--' , 'LineWidth',2.5)
plot(tout,rad2deg(Results.U_BL(2,:)),'g')
ylabel('\theta [deg]')
xlim([0 Parameter.Time.TMax])

subplot(613)
hold on;grid on; box on;
plot(tout,Results.X(6,:) , '--' , 'LineWidth',2.5)
plot(tout,Results.U_BL(1,:),'g')
ylabel('M_g [Nm]')
xlim([0 Parameter.Time.TMax])

subplot(614)
hold on;grid on; box on;
plot(tout,radPs2rpm(Results.X(1,:)), '--' , 'LineWidth',2.5)
plot(tout,radPs2rpm(Results.X_BL(1,:)),'g')
ylabel('\Omega [rpm]')
xlim([0 Parameter.Time.TMax])

subplot(615)
hold on;grid on; box on;
plot(tout,(Results.X(2,:)), '--' , 'LineWidth',2.5)
plot(tout,(Results.X_BL(2,:)),'g')
ylabel('x_T [m]')
xlim([0 Parameter.Time.TMax])


%figure;
subplot(616)
hold on;grid on; box on;
plot(tout,1e-6* M_yT_NMPC , '--' , 'LineWidth',2.5 )
plot(tout,1e-6*M_yT_BL ,'g')
ylabel('M_{yT} [MNm]')
xlim([0 Parameter.Time.TMax])

Load_emp = max(M_yT_NMPC)
Energy_emp = Energy_NMPC 
Energy_emp         = sum(P_el_NMPC)* 0.25 / 3.6e6
max_omega_NMPC      = max(Results.X(1,:));
min_omega_NMPC      = min(Results.X(1,:));
omega_emp=  max_omega_NMPC - min_omega_NMPC 



%%%%%%%%%%%%%%%---------------------------------------------------------------
 load('PSO_xx1.mat')
%  load('empirial_6.mat')
 

% subplot(611)
% hold on;grid on; box on;
% plot(Disturbance.v_0.time,Disturbance.v_0.signals.values, 'b', 'LineWidth',2.5 )
% ylabel('v_0 [m/s]')
% xlim([0 Parameter.Time.TMax])

 
subplot(612)
hold on;grid on; box on;
plot(tout,rad2deg(Results.X(4,:)) ,'r', 'LineWidth',1)
ylabel('\theta [deg]')
xlim([0 Parameter.Time.TMax])

subplot(613)
hold on;grid on; box on;
plot(tout,Results.X(6,:),'r', 'LineWidth',1)
ylabel('M_g [Nm]')
xlim([0 Parameter.Time.TMax])



subplot(614)
hold on;grid on; box on;
plot(tout,radPs2rpm(Results.X(1,:)) , 'r', 'LineWidth',1)
ylabel('\Omega [rpm]')
xlim([0 Parameter.Time.TMax])

subplot(615)
hold on;grid on; box on;
plot(tout,(Results.X(2,:)) ,'r', 'LineWidth',1)
ylabel('x_T [m]')
xlim([0 Parameter.Time.TMax])


%figure;
subplot(616)
hold on;grid on; box on;
plot(tout,1e-6* M_yT_NMPC ,'r', 'LineWidth',1)
ylabel('M_{yT} [MNm]')
xlim([0 Parameter.Time.TMax])

Load_pso = max(M_yT_NMPC)
Energy_pso = Energy_NMPC 

max_omega_NMPC      = max(Results.X(1,:));
min_omega_NMPC      = min(Results.X(1,:));
omega_pso= max_omega_NMPC - min_omega_NMPC 
Energy_pso         = sum(P_el_NMPC)* 0.25 / 3.6e6





