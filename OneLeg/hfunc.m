function F = hfunc(omega,A,b,k,W,forceRatio,h)
	F = (omega^2*b/(4*pi)*((pi/2 - asin(h/A))*A^2 - h*sqrt(A^2 - h^2))*(1 - forceRatio) ...
		- k/(2*pi)*((pi/2 - asin(h/A))*h - sqrt(A^2 - h^2))*(1 + forceRatio)) - W;
end
