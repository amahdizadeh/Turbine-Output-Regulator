
Trunc = 3:3600;

Wind    = Measurments.Data(Trunc,1);
Omega_R = Measurments.Data(Trunc,2);
Omega_G = Measurments.Data(Trunc,3);
Torsion = Measurments.Data(Trunc,8);
Gen_Trq = Measurments.Data(Trunc,7);



X0 = [ interp1(SSPoints.Wind , SSPoints.Omega_Gen, base_wnd_spd), ...
       interp1(SSPoints.Wind , SSPoints.Torsion , base_wnd_spd),...
       interp1(SSPoints.Wind , SSPoints.Omega_Rot , base_wnd_spd),...
       interp1(SSPoints.Wind , SSPoints.Torque_Gen , base_wnd_spd),...
       base_wnd_spd];

L = length(Wind);
X=[];
for i=1:L-1
    
    X = [X; [Omega_G(i)  Torsion(i)  Omega_R(i) Gen_Trq(i) Wind(i)]-X0];

end
y1 = Omega_G(2:end)-X0(1);    
y2 = Torsion(2:end)-X0(2);    
y3 = Omega_R(2:end)-X0(3);  

b1 = inv(X'*X)*X'*y1;
b2 = inv(X'*X)*X'*y2;
b3 = inv(X'*X)*X'*y3;


Ad = [b1(1:3)' ; b2(1:3)'; b3(1:3)']
Bd = [b1(4)' ; b2(4)'; b3(4)'];
Hd = [b1(5)' ; b2(5)'; b3(5)'];
Cd = [1 0 0 ];
Dd = 0;


Sysd = ss(Ad,Bd,Cd,Dd,.2);
opts = d2cOptions('Method','foh');
SysC = d2c(Sysd,opts);
eig(SysC.a)


