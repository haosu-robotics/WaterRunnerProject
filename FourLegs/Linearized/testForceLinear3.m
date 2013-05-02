%range%
y = 0.015;
theta = (-5:0.5:5 )* pi/180;
phi = (-5:0.5:5 )* pi/180;
w1 = 60;
w2 = w1;
[TH, PH] = meshgrid(theta,phi);

%params%
A = 0.0428;
r1 = 0.02;
r2 = r1/4;
area = 0.5449;
S1 = pi*r1^2*area;
S2 = pi*r2^2*area;
width = 0.125/2;
length = 0.125;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

amax = 5 * pi/180;
amin = -5 * pi/180;


while true
	theta0 = amin + (amax - amin).*rand();
	phi0 = amin + (amax - amin).*rand();

	F1 = Force(y,TH,PH,w1,w2,b_water,k_water,A,length,width,Fratio);
	F20 = Force(y,theta0,phi0,w1,w2,b_water,k_water,A,length,width,Fratio);
	F2 = F20 + Force_th(y,theta0,phi0,w1,w2,b_water,k_water,A,length,width,Fratio).*(TH - theta0) + ...
		Force_phi(y,theta0,phi0,w1,w2,b_water,k_water,A,length,width,Fratio).*(PH - phi0);

	surf(TH,PH,F1)
	hold on
	surf(TH,PH,F2)
	plot3(theta0,phi0,F20,'o','MarkerSize',5,'MarkerFaceColor','r')
	xlabel('theta')
	ylabel('phi')
	zlabel('F')
	hold off

	pause(1)
end
