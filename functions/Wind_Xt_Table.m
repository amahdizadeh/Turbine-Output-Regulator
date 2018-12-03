function out = Wind_Xt_Table(Wind_Speed)

% wind   ,  theta
Wind_Xt = [
  11.2,      0.3648
  12,        0.3148
  13,        0.2697
  14,        0.2415
  15,        0.2215
  16,        0.2040
  17,        0.1904  
  18,        0.1821
  19,        0.1720
  20,        0.1670 
  21,        0.1623
  22,        0.1545  
  23,        0.1516
  24,        0.1465
  25,        0.1449];

wind = Wind_Xt(:,1);
Xt = Wind_Xt(:,2);


out = interp1(wind,Xt,Wind_Speed,'spline');

end