function wss = findSSinput(y, b, k, A, alpha, Weight, w0)
%Solve force function for input that will create an average force equal to the weight

%y, theta, phi = state variables
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%alpha = forceRatio
%w0 initial guess for steady state input

h = y - A;
F = @(w)Force(w, h, b, k, A, alpha, Weight);
options = optimset('Display','off'); %solver options
wss = fsolve(F,w0,options);

end

function F = Force(w, h, b, k, A, alpha, Weight)
	%Force function
	dragPart = b*w.*w.*(w - w*alpha).*(A^2 * acos(h/A) - ... 
		h*sqrt(A^2 - h^2))./(2*pi*(w + w));
		
	springPart = k*(w + w*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(2*pi*w);
	F = dragPart + springPart - Weight;
end
