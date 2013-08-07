function F_w1 = Force_w1(y,theta,phi,w1,w2,b,k,A,l,w,alpha)
%FORCE_W1
%    F_W1 = FORCE_W1(Y,THETA,PHI,W1,W2,B,K,A,L,W,ALPHA)

%    This function was generated by the Symbolic Math Toolbox version 5.9.
%    02-May-2013 13:49:21

t3 = sin(phi);
t4 = l.*t3;
t5 = sin(theta);
t6 = t5.*w;
t2 = A-t4+t6-y;
t7 = 1.0./pi;
t8 = t2.^2;
t9 = A.^2;
t10 = -t8+t9;
t11 = sqrt(t10);
t12 = 1.0./A;
t13 = t2.*t12;
t14 = acos(t13);
t15 = pi-t14;
t16 = t2.*t15;
t17 = t11+t16;
t18 = t2.*t11;
t19 = t9.*t15;
t20 = t18+t19;
t21 = w1+w2;
t22 = 1.0./t21;
t23 = w1-alpha.*w2;
F_w1 = k.*t7.*t17.*1.0./w1.^2.*(w2+alpha.*w1).*(-1.0./2.0)+(alpha.*k.*t7.*t17.*(1.0./2.0))./w1+b.*t7.*t20.*t22.*t23.*w2.*(1.0./2.0)+b.*t7.*t20.*t22.*w1.*w2.*(1.0./2.0)-b.*t7.*t20.*1.0./t21.^2.*t23.*w1.*w2.*(1.0./2.0);
