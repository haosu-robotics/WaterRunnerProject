mass = 0.05;

Amp = 0.0249;
amp = Amp;
omega_0 = [60 60]';
leg_length = Amp;

freq = 70;
RL_lag = pi;
W_coupling = 1;

y_0 = leg_length/2;

r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2;
S2 = pi*r2^2;

density = 1000;
g = 9.81;
C_d = 0.703;
area= 0.9185;
S1 = (pi*r1^2)*area;
S2 = (pi*r2^2)*area;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
FRatio = 1/16;

tau = 0.001;
alpha = 30;
t_fil = 0.2;
