global r1 forceRatio mass DF

%T_des = 20;
mass = .152/4;
numLegs = 4;

%Amp = 0.0268; Calibrated to V1 sim
Amp = 0.0428; %Calibrated to V2 sim
amp = Amp;
leg_length = Amp;
freq = 84;
DF = 0.5;
y_0 = Amp/2;

r1 = 0.02;
r2 = r1/4;
forceRatio = (r2^2)/(r1^2);
%area= 0.9185; Calibrated to V1 sim
area= .5449; %Calibrated to V2 sim
S1 = (pi*r1^2)*area;
S2 = (pi*r2^2)*area;

density = 1000;
g = 9.81;
C_d = 0.703;

tau = 0.001;
alpha = 30;
