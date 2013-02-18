function P =  SSpower(h,omega,A,radius,forceRatio)
	
	C_d = 0.703;
	density = 1000;
	g = 9.81;
	S = pi*radius^2;

	b = 0.5*C_d*S*density;
	k = C_d*S*density*g;

	P = ((b*omega^3)/(2*pi))*(1 + forceRatio)*((h^3)/3 + (2*A^3)/3 - (A^2)*h) + (k*omega/(4*pi))*(1 - forceRatio)*(A - h)^2;
end
