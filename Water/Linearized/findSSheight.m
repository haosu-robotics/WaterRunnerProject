function wss = findSSheight(w1, w2, b, k, A, alpha, Weight, h0)
%Solve force function for steady state height for a given w1 and w2 
%steady state height will create an average force equal to the weight

%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%alpha = forceRatio
%h0 initial guess for steady state height

F = @(h)Force(h, w1, w2, b, k, A, alpha, Weight)
%options = optimset('Display','off'); %solver options
wss = fsolve(F,h0); %,options);

end

function F = Force(h, w1, w2, b, k, A, alpha, Weight)
	%Force function
	dragPart = (b/(2*pi))*((w1.^2).*w2 - w1.*(w2.^2)*alpha).*(A^2 * acos(h/A) - ... 
		h.*sqrt(A^2 - h.^2))./(w1 + w2);
		
	springPart = k*(w2 + w1*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(pi*(w1 + w2));
	F = dragPart + springPart - Weight;
end
