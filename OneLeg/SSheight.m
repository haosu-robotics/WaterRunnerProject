function h =  SSheight(omega,A,radius,m,forceRatio,numLegs)
	
	C_d = 0.703;
	density = 1000;
	g = 9.81;
	S = pi*radius^2;

	b = 0.5*C_d*S*density;
	k = C_d*S*density*g;
	
	W = m*g*numLegs;
	
	fun = @(h)hfunc(omega,A,b,k,W,forceRatio,numLegs,h);
	h0 = A/2;
	
	h = fsolve(fun,h0);
end
