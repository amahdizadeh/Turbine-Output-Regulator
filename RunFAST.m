clear; clc;

addpath('Tools');
addpath('Lib');
load_system('NREL5MW_Library.mdl')  

%% load neccesary data
load('TestProfile.mat')
load('WindSet_22Mar.mat')


%% Run initialization of Simulink model
% Default
Parameter                               = NREL5MWDefaultParameterFAST;

% Modifications     
Parameter.FF.Mode                       = 0; 

% Simulink
Parameter.Simulink.ModelName            = 'NREL5MW_General';

% Time
Parameter.Time.TMax                     = TestProfile.Tmax  ;

% Wind
Creat_hh_File(WindSet,TestProfile.Turbulance_index,TestProfile.WindSpeed,'NREL5MW_GeneralWind.hh');
Wind_LIDAR                              = WindSet{TestProfile.Turbulance_index}{TestProfile.WindSpeed}.LIDAR;
Wind_Real                               = WindSet{TestProfile.Turbulance_index}{TestProfile.WindSpeed}.Effective;

Disturbance.v_0.time                    = Wind_LIDAR(:,1);
Disturbance.v_0.signals.values         	= Wind_LIDAR(:,2);
Disturbance.v_0L.time                   = Disturbance.v_0.time-Parameter.Scan.T_preview;
Disturbance.v_0L.signals.values         = Disturbance.v_0.signals.values;


% Setup
if (TestProfile.WindSpeed < 11.4)
    Setup_Region2;
else
    Setup_Region3;
end

% Secondary
Parameter                               = NREL5MWSecondaryParameterFAST(Parameter,Disturbance);

%% Run initialization for FAST sFunction
input_fast = 'NREL5MW_ClassA.fst';
Read_FAST_Input;
if strcmp(OutList(1),'OutList'); OutList(1)=[];end % strange 64bit behaviour

% Set Initial Condidtions
if isfield(Parameter.IC,'q') && isfield(Parameter.IC,'qdot')
    q_init      = Parameter.IC.q;
    qdot_init   = Parameter.IC.qdot;   
end


%% Simulate FAST
try
    sim(Parameter.Simulink.ModelName)
catch exception
    disp(exception.message)
    eval([Parameter.FAST.SFuncName,'(0,[],[],9)']);
end


%% DEL Calculation
CalculateDEL_MyT
CalculateDEL_MxT
CalculateDEL_Myb
CalculateDEL_Mxb
CalculateDEL_LSS

Energy    = sum(Rec(fix(START/dt):fix(END/dt),13))*dt;
Power_STD = std(Rec(fix(START/dt):fix(END/dt),13));
Omega_STD = std(Rec(fix(START/dt):fix(END/dt),2));
CTR_RMS   = rms((diff((Rec(fix(START/dt):fix(END/dt),6))/dt)));

Omega = Rec(fix(START/dt):fix(END/dt),2);
v0 = Rec(fix(START/dt):fix(END/dt),1);
Lambda = Omega.*Parameter.Turbine.R./v0;
Lambda_STD = std(Omega.*Parameter.Turbine.R./v0);

%% Record Data
Methods = {'EOR','BL','DAC'};
if (TestProfile.UpdateRecords == 1)
    if (TestProfile.WindSpeed < 11.4)
        Region2_Record(TestProfile.WindSpeed,TestProfile.CtlType,...
                    [DEL.MyT.Value, DEL.MxT.Value, Lambda_STD, Power_STD, DEL.Myb.Value , DEL.Mxb.Value, DEL.LSS.Value, Energy],...
                    '../ReportMaker/Simulation_Results.xlsx')
       
        fname = strcat('../ReportMaker/Region2_Temporal_',Methods{TestProfile.CtlType},'.mat');
        copyfile ('Sampled_Data.mat', fname);
    end
end
Experiment_Record([TestProfile.WindSpeed, 0 ,TestProfile.CtlType, DEL.MyT.Value, DEL.MxT.Value, Lambda_STD, Power_STD,...
                        DEL.Myb.Value , DEL.Mxb.Value, DEL.LSS.Value, Energy,...
                            TestProfile.ResampleRef_Ts,TestProfile.ResampleDis_Ts, TestProfile.LQR_R, Ts, TestProfile.LGain],...
                   '../ReportMaker/Simulation_Results.xlsx');


        
%% variable update
[num,txt,raw] = xlsread('../ReportMaker/Simulation_Results.xlsx',5);
for i=3:5
    for j=4:11
        txt{i,j} = num(i-2,j);
    end
    txt{i,12}=' ';
end
txt{2+TestProfile.CtlType ,12}='<';


%% Plot

subplot(611)
plot( Wind_Real(:,1), Wind_Real(:,2))
xlim([0 Parameter.Time.TMax])
ylabel('v_0 [m/s]')

subplot(612)
hold on;grid on;
plot(Measurments.Time,Measurments.Data(:,2))
hold on
plot(Refs.Time,Refs.Data(:,1)+working_point(1),'Color','Black','LineStyle' ,'-.')
ylabel('Omega [Rad/s]')

subplot(613)
hold on;grid on
plot(Measurments.Time,Measurments.Data(:,4))
ylabel('x_T [m]')
subplot(627)
hold on;grid on
plot(Measurments.Time,Measurments.Data(:,5))
ylabel('Theta [Rad]')
subplot(628)
hold on;grid on
plot(Measurments.Time, Measurments.Data(:,2).*Parameter.Turbine.R./Measurments.Data(:,1));
ylabel('Lambda')
subplot(615)
hold on;grid on
plot(Measurments.Time,Measurments.Data(:,7))
ylabel('Gen Torque')
subplot(616)
hold on;grid on
plot(Measurments.Time,Measurments.Data(:,8))
ylabel('Torsion [Rad]')
hold on;grid on



