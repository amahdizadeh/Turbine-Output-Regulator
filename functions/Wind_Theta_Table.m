function out = Wind_Theta_Table(WindS_Speed)

% wind (m/sec)  ,  theta  (deg)
Wind_Theta = [
  11.2,       0
  12,        3.8
  13,       6.6
  14,        8.6
  15,        10.45
  16,        12.05
  17,        13.54  
  18,        14.92
  19,        16.22
  20,        17.47 
  21,        18.7
  22,        19.95   
  23,        21.17
  24,        22.34
  25,        23.47];

wind = Wind_Theta(:,1);
thta = Wind_Theta(:,2);


out = interp1(wind,thta,WindS_Speed,'spline');

end