mass = .1258/2
theta1 = 0;
theta2 = pi;

L1 = .0615;   %distance between each motor and its passive joint
L2 = 0.0218;
L3 = 0.0748;
L4 = 0.0468;
L5 = 0.0624;

alpha =  2*L1*L4*cos(theta1) - 2*L2*L4*cos(theta2);
beta  = -2*L1*L4*sin(theta1) - 2*L2*L4*sin(theta2);
gamma = L1^2 + L4^2 + L2^2 - L3^2 - 2*L1*L2*cos(theta1)*cos(theta2) - 2*L1*L2*cos(theta1)*cos(theta2);

delta = atan2(beta, alpha);
theta4 = delta + acos(-gamma/(norm([alpha,beta])))
theta3  = atan2( L1*sin(theta1) + L4*sin(theta4)-L2*sin(theta1) , L1*cos(theta1) + L4*cos(theta4) - L2*cos(theta2))


