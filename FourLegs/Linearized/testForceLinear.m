%range%
y = 0.01;
w1 = 0:5:100;
w2 = w1;
[W1, W2] = meshgrid(w1,w2);

%params%
A = 0.0249;
r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2*0.9185;
S2 = pi*r2^2*0.9185;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

wmax = 80;
wmin = 10;

while true
	w10 = wmin + (wmax - wmin).*rand();
	w20 = wmin + (wmax - wmin).*rand();

	F1 = Force(y,W1,W2,b_water,k_water,A,Fratio);
	F20 = Force(y,w10,w20,b_water,k_water,A,Fratio);
	F2 = F20 + Force_w1(y,w10,w20,b_water,k_water,A,Fratio).*(W1 - w10) + ...
		Force_w2(y,w10,w20,b_water,k_water,A,Fratio).*(W2 - w20);

	surf(W1,W2,F1)
	hold on
	surf(W1,W2,F2)
	plot3(w10,w20,F20,'o','MarkerSize',5,'MarkerFaceColor','r')
	xlabel('w1')
	ylabel('w2')
	zlabel('F')
	hold off

	pause(1)
end
