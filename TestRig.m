%%  initialization
TestProfile.CtlType             = 3;    % 1:EOR % 2:PID  3:DAC
TestProfile.Tmax                = 900;  
TestProfile.WindSpeed           = 11;
TestProfile.SpeedStep           = 1;
TestProfile.Turbulance_index    = 1;    % 1:Class A % 2:Class B % 3:Class C
TestProfile.UpdateRecords       = 1;



%% Controller parameters
    TestProfile.StateEstimation     = 1;    % 0:State Feedback   %   1:Measurment Feedback     
if (TestProfile.CtlType ==1)  %% EOR
    TestProfile.ResampleDis_Ts      = 8;   % DAC: 0.5    EOR:4
    TestProfile.ResampleRef_Ts      = 8;    % DAC: 8.0    EOR:8      
    TestProfile.CLPoles             = [ -0.0731*6.5  -2.0958-15.6645i  -2.0958+15.6645i]; % poles
    TestProfile.LQR_R               = 5e-10; %7:3e-10    %3e-10;
else    
    TestProfile.ResampleDis_Ts      = .5;   % DAC: 0.5    EOR:4
    TestProfile.ResampleRef_Ts      = 8; %8;    % DAC: 8.0    EOR:8  
    TestProfile.CLPoles             = [ -0.0731*2  -2.0958-15.6645i  -2.0958+15.6645i];   % poles
    TestProfile.LQR_R               = 1e-9;    %2e-10;
end
%EOR : (works against DAC)[ -0.0731*4.0  -2.0958-15.6645i  -2.0958+15.6645i];  [ -0.0731*6  -0.8958-15.6645i  -0.8958+15.6645i];
%DAC : [ -0.0731*3.0  -5.8958-15.6645i  -5.8958+15.6645i];

TestProfile.LQR_Q  = LWT.Cy' * LWT.Cy;
% TestProfile.LQR_Q               = [1e2   0   0;
%                                     0  1e-4  0;
%                                     0    0  1e2];
% TestProfile.LQR_R               =  1e2;  %1e4
TestProfile.LGain               =  1;
TestProfile.UseLQR              =  1;

%% Measurment parameters
TestProfile.DEL_Start           = 100;


% load 'TestProfile' 
save 'TestProfile' TestProfile
RunFAST

% while (TestProfile.WindSpeed <26 )  
%     RunFAST_DAC_R3
%     TestProfile.WindSpeed = TestProfile.WindSpeed + 1;
%     save 'TestProfile' TestProfile
% end

% % while (TestProfile.Turbulance_index <4)
%     while (TestProfile.WindSpeed <26 )        
%         while (TestProfile.CtlType <3 )        
%             RunFAST_R3
%             TestProfile.CtlType = TestProfile.CtlType + 1;
%             save 'TestProfile' TestProfile    
%         end
%         TestProfile.CtlType = 1; 
%         TestProfile.WindSpeed = TestProfile.WindSpeed + 1;
%         if (TestProfile.WindSpeed==22)
%             TestProfile.WindSpeed = 23;   % skip the unstable part
%         end
%         save 'TestProfile' TestProfile  
%     end
%     TestProfile.CtlType = 1;
%     TestProfile.WindSpeed = 12;
%     TestProfile.Turbulance_index = TestProfile.Turbulance_index + 1;
%     save 'TestProfile' TestProfile  
% end



% TestProfile.LQR_Q               = [1e0   0   0;
%                                     0  1e-4  0;
%                                     0    0  1e0];





