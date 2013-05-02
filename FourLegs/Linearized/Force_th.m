function Fth = Force_th(y,w1,w2,b,k,A,l,w,alpha)

%y, theta, phi = state variables
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%l = lengthwise distance from COM
%w = widthwise distance from COM
%alpha = forceRatio


h = height(y,theta,phi,l,w,A);

dragPart = b*w1*w2*(w1 - w2*alpha) * ...
	((-w * cos(theta)*h.^2)./sqrt(A^2 - h.^2) ... 
	+ w*cos(theta) * sqrt(A^2 - h.^2) + ...
	A*w*cos(theta)./sqrt(1 - (h/A).^2))./(2*pi*(w1 + w2));

springPart = k*(w2 + w1*alpha)*(w.*acos(h/A)*cos(theta) + ...
	(w.*cos(theta).*h)./sqrt(A^2 - h^2) - ...
	(w.*cos(theta).*h)./(A.*sqrt(1 - (h/A).^2)))./(2*pi*w1);

Fth = dragPart + springPart;
