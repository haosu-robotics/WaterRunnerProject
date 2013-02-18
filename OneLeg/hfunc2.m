function F = hfunc(omega,A,b,k,W,forceRatio,numLegs,h)
	F = numLegs*(omega^2*b/(4*pi)*((pi/2 - (h/A))*A^2 - h*A)*(1 - forceRatio) ...
		- k/(2*pi)*((pi/2 - (h/A))*h - A)*(1 + forceRatio)) - W;
end
