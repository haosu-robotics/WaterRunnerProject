mass = 0.05;
leg_length = 0.06;

% duty_factor = 0.3;
amp = leg_length;
freq = 60;

y_0 = leg_length;

r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2;
S2 = pi*r2^2;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
FRatio = 1/16;


tau = 0.001;
alpha = 30;
