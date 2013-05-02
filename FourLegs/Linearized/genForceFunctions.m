%y, theta, phi = state variables
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%l = lengthwise distance from COM
%w = widthwise distance from COM
%alpha = forceRatio
syms y theta phi b k A l w alpha h(y,theta,phi,l,w,A)

%height function 
h(y,theta,phi,l,w,A) =  y + l*sin(theta) - w*sin(phi) - A;

dragPart = b*w1.*w2.*(w1 - w2*alpha).*(A^2 * acos(h/A) - ... 
	h*sqrt(A^2 - h^2))./(2*pi*(w1 + w2));
	
springPart = k*(w2 + w1*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(2*pi*w1);

F  = dragPart + springPart;
Fy = 

