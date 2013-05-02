function Fw1 = Force_w1(y,w1,w2,b,k,A,l,w,alpha)

%y, theta, phi = state variables
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%l = lengthwise distance from COM
%w = widthwise distance from COM
%alpha = forceRatio

h = height(y,theta,phi,l,w,A);

Fw1 = k*alpha*(acos(h/A).*(-1)*h + sqrt(A^2 - h.^2))./(2*pi*w1) - ...
	k*(w2 + w1*alpha).*((acos(h/A) .*(-1)*h)+ ...
	sqrt(A^2 - h.^2))./(2*pi*w1.^2)  - ...
	b* w1 .* w2 .* (w1 - w2*alpha) .* (A^2 .* acos(h/A) - ...
	h*sqrt(A.^2 - h.^2)) + ...
	b*w2.*(2*w1 - w1*alpha).*(A^2 * acos(h/A) - h.*sqrt(A^2 - h.^2));
