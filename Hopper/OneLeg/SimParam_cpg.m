global r1 Fratio mass DF

%T_des = 20;
mass = 0.025;
numLegs = 4;

Amp = 0.0249;
amp = Amp;
omega_0 = [60 60]';
leg_length = Amp;
freq = 84;
DF = 0.5;
y_0 = Amp/2;

density = 1000;
g = 9.81;
C_d = 0.703;

r1 = 0.02;
r2 = r1/4;
Fratio = (r2^2)/(r1^2);
area= 0.9185;
S1 = (pi*r1^2)*area;
S2 = (pi*r2^2)*area;
b = 0.5*C_d*S1*density;
k = C_d*S1*density*g;

tau = 0.001;
alpha = 30;
t_fil = 0.2;
