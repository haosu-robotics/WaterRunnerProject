%range%
y = 0:0.001:0.04;
w1 = 60;
w2 = w1;

%params%
%A = 0.0249;
A = 0.0428;
r1 = 0.02;
r2 = r1/4;
area = 0.5449;
S1 = pi*r1^2*area;
S2 = pi*r2^2*area;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

ymax = 0.04;
ymin = 0;

while true
	y0 = ymin + (ymax - ymin).*rand();

	F1 = Force(y,w1,w2,b_water,k_water,A,Fratio)
	F20 = Force(y0,w1,w2,b_water,k_water,A,Fratio);
	F2 = F20 + Force_y(y0,w1,w2,b_water,k_water,A,Fratio).*(y - y0);

	plot(y,F1)
	hold on
	plot(y,F2)
	plot(y0,F20,'o','MarkerSize',5,'MarkerFaceColor','r')
	xlabel('y')
	ylabel('F')
	hold off

	pause(1)
end
