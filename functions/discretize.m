function out = discretize(Sys, T)

sysd = c2d(ss(Sys.A,Sys.B,Sys.C,Sys.D),T);

LWTD.A = sysd.a;
LWTD.B = sysd.b;
LWTD.B_d = Sys.B_d * T;
LWTD.C = sysd.c;
LWTD.D = sysd.d;
LWTD.Cy = Sys.Cy;
LWTD.Dy = Sys.Dy;
LWTD.Ts = T;
out = LWTD;
