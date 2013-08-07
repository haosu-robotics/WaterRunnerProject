function F_w2 = Force_w2(y,theta,w1,w2,b,k,A,w,alpha)
%FORCE_W2
%    F_W2 = FORCE_W2(Y,THETA,W1,W2,B,K,A,W,ALPHA)

%    This function was generated by the Symbolic Math Toolbox version 5.9.
%    02-May-2013 21:12:12

t2 = sin(theta);
t3 = t2.*w;
t4 = A+t3-y;
t5 = 1.0./pi;
t6 = A.^2;
t7 = 1.0./A;
t8 = t4.*t7;
t9 = acos(t8);
t10 = pi-t9;
t11 = t4.^2;
t12 = t6-t11;
t13 = sqrt(t12);
t14 = w1-alpha.*w2;
t15 = w1+w2;
t16 = t6.*t10;
t17 = t4.*t13;
t18 = t16+t17;
t19 = 1.0./t15;
F_w2 = real((k.*t5.*(t13+t4.*t10).*(1.0./2.0))./w1+b.*t5.*t14.*t18.*t19.*w1.*(1.0./2.0)-alpha.*b.*t5.*t18.*t19.*w1.*w2.*(1.0./2.0)-b.*t5.*t14.*1.0./t15.^2.*t18.*w1.*w2.*(1.0./2.0));
