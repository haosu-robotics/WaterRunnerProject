function h =  SSheight3(coeff,omega)
	%version of steady state height equation for least squares fitting to simulated robot data

	global r1 forceRatio mass

	Amp = coeff(1);
	area = coeff(2);

	C_d = 0.703;
	density = 1000;
	g = 9.81;
	S = area*(pi*r1^2);

	b = 0.5*C_d*S*density;
	k = C_d*S*density*g;
	
	W = mass*g;
	
	C1 = (omega.^2)*b*Amp*(1/.DF-forceRatio./(1-DF))/(8*pi);
	C2 = k*(DF + forceRatio*(1- DF))/pi;

	A = C2./Amp;
	B = -1*(2*C1 + C2*pi/2);
	C = C1*pi.*Amp/2 + C2.*Amp - W;

	h = (-B - sqrt(B.^2 - 4*A.*C))./(2*A) + Amp;
	h(h<0) = 0;
end
