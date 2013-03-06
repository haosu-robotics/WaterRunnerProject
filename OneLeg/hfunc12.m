function F = hfunc12(omega,DF,A,b,k,W,forceRatio,h)

	B = b*(A^2)*(acos(h/A) - h*sqrt(A^2 - h^2)/(A^2))/(2*pi);
	K = k*(sqrt(A^2 - h^2) - h*acos(h/A))/pi;
	
	w1 = omega/(2*DF);
	w2 = omega/(2*(1 - DF));

	F = B*(w1^2 * w2 - forceRatio* w1 * w2^2)/(w1 + w2) + K * (w2 + forceRatio * w1)/(w1 + w2) - W;
end
