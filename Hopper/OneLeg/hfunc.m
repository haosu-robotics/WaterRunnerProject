function F = hfunc(omega,DF,A,b,k,W,forceRatio,h)
	F = (omega^2*b/(8*pi)*((pi/2 - asin(h/A))*A^2 - h*sqrt(A^2 - h^2))*(1/DF - forceRatio/(1-DF)) ...
		- k/(pi)*((pi/2 - asin(h/A))*h - sqrt(A^2 - h^2))*(DF + forceRatio*(1-DF))) - W;
end
