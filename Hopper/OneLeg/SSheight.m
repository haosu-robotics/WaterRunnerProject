function h =  SSheight(omega,DF,A,radius,m,forceRatio,area)
	%numerical solution to steady state height
	C_d = 0.703;
	density = 1000;
	g = 9.81;
	S = area*(pi*radius^2);

	b = 0.5*C_d*S*density;
	k = C_d*S*density*g;
	
	W = m*g;
	
	fun = @(h)hfunc(omega,DF,A,b,k,W,forceRatio,h);
	h0 = A/2;
	
	h = fsolve(fun,h0);
end
