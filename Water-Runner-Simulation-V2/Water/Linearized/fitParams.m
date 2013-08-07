function hvec =  fitParams(coeff,omega)
	%version of steady state height equation for least squares fitting to simulated robot data
	global mass

	A = coeff(1);
	area = coeff(2);

	C_d = 0.703;
	density = 1000;
	g = 9.81;
	r1 = 0.02;
	S = area*(pi*r1^2);
	forceRatio = 1/16;

	b = 0.5*C_d*S*density;
	k = C_d*S*density*g;
	
	W = mass*g/4;
	
	hvec = zeros(size(omega));
	i = 1;
	for w = omega
		fun = @(h)Force(h, w, w, b, k, A, forceRatio, W)
		h0 = A/2;
		hvec(i) = fsolve(fun,h0) + A;
		i = i + 1;
	end
end

function F = Force(h, w1, w2, b, k, A, alpha, Weight)
	%Force function
	dragPart = (b/(2*pi))*((w1.^2).*w2 - w1.*(w2.^2)*alpha).*(A^2 * acos(h/A) - ... 
		h.*sqrt(A^2 - h.^2))./(w1 + w2);
		
	springPart = k*(w2 + w1*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(pi*(w1 + w2));
	F = dragPart + springPart - Weight;
end
