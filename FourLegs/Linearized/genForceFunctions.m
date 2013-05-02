%y, theta, phi = state variables
%w1, w2 = system inputs
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%l = lengthwise distance from COM
%w = widthwise distance from COM
%alpha = forceRatio
syms y theta phi w1 w2 b k A l w alpha h(y,theta,phi,l,w,A)

%height function 
h(y,theta,phi,l,w,A) =  y + l*sin(theta) - w*sin(phi) - A;

dragPart = b*w1.*w2.*(w1 - w2*alpha).*(A^2 * acos(h/A) - ... 
	h*sqrt(A^2 - h^2))./(2*pi*(w1 + w2));
	
springPart = k*(w2 + w1*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(2*pi*w1);

F  = dragPart + springPart
Fy = diff(F,y);
Fth = diff(F,theta)
Fphi = diff(F,phi)
Fw1 = diff(F,w1);
Fw2 = diff(F,w2);

matlabFunction(F,'file','Force.m','outputs',{'F'},'vars',[y, theta, phi, w1, w2, b, k, A, l, w, alpha]);
matlabFunction(Fy,'file','Force_y.m','outputs',{'F_y'},'vars',[y, theta, phi, w1, w2, b, k, A, l, w, alpha]);
matlabFunction(Fth,'file','Force_th.m','outputs',{'F_th'},'vars',[y, theta, phi, w1, w2, b, k, A, l, w, alpha]);
matlabFunction(Fphi,'file','Force_phi.m','outputs',{'F_phi'},'vars',[y, theta, phi, w1, w2, b, k, A, l, w, alpha]);
matlabFunction(Fw1,'file','Force_w1.m','outputs',{'F_w1'},'vars',[y, theta, phi, w1, w2, b, k, A, l, w, alpha]);
matlabFunction(Fw2,'file','Force_w2.m','outputs',{'F_w2'},'vars',[y, theta, phi, w1, w2, b, k, A, l, w, alpha]);
