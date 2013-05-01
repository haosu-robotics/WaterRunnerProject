function Fw1 = Force_w1(y,w1,w2,b,k,A,alpha)

%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%alpha = forceRatio

Fw1 = -1*w2.*(sqrt((2*A - y)*y) .* ...
	(k*(w1 + w2).^2 - b*w1.^2 .*(A-y).*(w1.^2 + 2*w1.*w2 - w2.^2 * alpha)) + ...
	(A*k*(w1 + w2).^2 - k*y.*(w1 + w2).^2 - A^2 * b * w1.^2 * (w1.^2 + 2* w1 .* w2 - w2.^2 *alpha)) * ...
	acos(y/A - 1))./(2*pi*(w1.^2) .* (w1 + w2).^2);
