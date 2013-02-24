global r1 forceRatio mass

mass = 0.025;
numLegs = 4;

Amp = 0.0260;
leg_length = Amp;
%freq = 70;
%y_0 = Amp/2;

r1 = 0.02;
r2 = r1/4;
forceRatio = (r2^2)/(r1^2);
area= 0.7968;
S1 = (pi*r1^2)*area;
S2 = (pi*r2^2)*area;

density = 1000;
g = 9.81;
C_d = 0.703;

