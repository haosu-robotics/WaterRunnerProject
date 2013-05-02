function F = Force(y,theta,phi,w1,w2,b,k,A,l,w,alpha)

%y, theta, phi = state variables
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%l = lengthwise distance from COM
%w = widthwise distance from COM
%alpha = forceRatio

h = height(y,theta,phi,l,w,A);

dragPart = b*w1.*w2.*(w1 - w2*alpha).*(A^2 * acos(h/A) - ... 
	h*sqrt(A^2 - h^2))./(2*pi*(w1 + w2));
	
springPart = k*(w2 + w1*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(2*pi*w1);

F = dragPart + springPart;
