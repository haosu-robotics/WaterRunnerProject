%y, theta, phi = state variables
%w1, w2 = system inputs
%b = damping constant
%k = spring constant
%A = amplitude = leg_length
%l = lengthwise distance from COM
%w = widthwise distance from COM
%alpha = forceRatio
syms y theta w1 w2 b k A w alpha Weight h(y,theta,w,A)

%height function 
h(y,theta,w,A) =  y - w*sin(theta) - A;

dragPart = b*w1.*w2.*(w1 - w2*alpha).*(A^2 * acos(h/A) - ... 
	h*sqrt(A^2 - h^2))./(2*pi*(w1 + w2));
	
springPart = k*(w2 + w1*alpha) .* (-1 * h .* acos(h/A) + sqrt(A^2 - h.^2))./(2*pi*w1);

F  = dragPart + springPart;
Fy = diff(F,y);
Fth = diff(F,theta);
Fw1 = diff(F,w1);
Fw2 = diff(F,w2);

matlabFunction(F,'file','Force.m','outputs',{'F'},'vars',[y, theta, w1, w2, b, k, A, w, alpha]);
matlabFunction(Fy,'file','Force_y.m','outputs',{'F_y'},'vars',[y, theta, w1, w2, b, k, A, w, alpha]);
matlabFunction(Fth,'file','Force_th.m','outputs',{'F_th'},'vars',[y, theta, w1, w2, b, k, A, w, alpha]);
matlabFunction(Fw1,'file','Force_w1.m','outputs',{'F_w1'},'vars',[y, theta, w1, w2, b, k, A, w, alpha]);
matlabFunction(Fw2,'file','Force_w2.m','outputs',{'F_w2'},'vars',[y, theta, w1, w2, b, k, A, w, alpha]);
