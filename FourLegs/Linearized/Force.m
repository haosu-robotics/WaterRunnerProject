function F = Force(y,w1,w2,b,k,A,alpha)

%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%alpha = forceRatio

dragPart = b.*w1.*w2 .* (w1 - w2*alpha).*((A - y) .* sqrt((2*A - y).*y) + A^2 .* acos(y/A - 1))./(2.*pi.*(w1 + w2));

springPart = k.*(w2 + w1.*alpha).*(sqrt((2*A - y).*y) + (A - y).*acos(y./A -1))./(2.*pi.*w1);

F = dragPart + springPart;
