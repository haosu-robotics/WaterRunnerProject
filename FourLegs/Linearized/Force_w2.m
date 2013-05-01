function Fw2 = Force_w2(y,w1,w2,b,k,A,alpha)

%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%alpha = forceRatio

Fw2 = (sqrt((2*A - y).*y) .* ...
	(k*(w1 + w2).^2 + b*w1.^2 .*(A-y).*(w1.^2 - 2*w1.*w2*alpha - w2.^2 * alpha)) + ...
	(A*k*(w1 + w2).^2 - k*y.*(w1 + w2).^2 + A^2 * b * w1.^2 * (w1.^2 - 2* w1 .* w2 * alpha - w2.^2 *alpha)) * ...
	acos(y/A - 1))./(2*pi*w1.*(w1 + w2).^2);
