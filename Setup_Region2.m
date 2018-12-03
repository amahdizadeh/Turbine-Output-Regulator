load('5MW_SteadyState_Torsion.mat')


% initials
Ts =.25 %.2;

base_wnd_spd = TestProfile.WindSpeed;

working_point = [ interp1(SSPoints.Wind , SSPoints.Omega_Rot, base_wnd_spd), ...
                  interp1(SSPoints.Wind , SSPoints.Torsion , base_wnd_spd),...
                  interp1(SSPoints.Wind , SSPoints.Omega_Gen, base_wnd_spd)];

%

LWT = Linearize_5MW('Wright',base_wnd_spd); 
LWTd = discretize(LWT,Ts)
%% DAC Gain calculation
B  =  LWT.B;
B_d = LWT.B_d;
func = @(g)(norm(norm(B)*g-norm(B_d)));
DAC_G = fminunc(func,0)


%% Close loop poles
[K,S,E] = lqr(LWT.A,LWT.B,TestProfile.LQR_Q,TestProfile.LQR_R);

if (TestProfile.UseLQR == 0)
    FBK = -place(LWT.A,LWT.B,TestProfile.CLPoles);
else
    FBK = -K;
end
%eig(LWT.A-LWT.B*FBK)


% Wind
CorrectedWind = Wind_LIDAR;
CorrectedWind(:,2) =  CorrectedWind(:,2) - base_wnd_spd;



%% Exo Systems
Exo.S1 = [0   1   0 ;
          0   0   1 ;          
         -4  -3  -2 ];        % disturbance exo system
Exo.L1 = [1  1  1 ];

Exo.S2 = [0  .25   0;
          -.25  0  0;
          0     0  0];        % reference exo system
      
Exo.L2 = [0.1  .8  0]*.1;


build_dataTypes;
