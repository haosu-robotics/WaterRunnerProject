%%%%% Initial State %%%%%%
y0 = 0.020;
theta0 = 0;
phi0 = 0;
x0 = [y0; 0; theta0; 0; phi0; 0; 1];

%%%%% Desired State %%%%%%
y = 0.020;
theta = 0 * pi/180;
phi = 0 * pi/180;
w = 60;
xdes = [y, theta, phi];

%%%%% Robot Params %%%%%
A = 0.0428;
r1 = 0.02;
r2 = r1/4;
area = 0.5449;
S1 = pi*r1^2*area;
S2 = pi*r2^2*area;

mass = 0.15;
Ix = 2.0114e-04;
Iz = 7.8949e-04;

width  = 0.125/2; 
length = 0.25/2;
M = [mass 0 0; 0 Ix 0 ; 0 0 Iz]; %mass matrix
  
marm_x = [1 1 -1 -1]';
marm_z = [1 -1 -1 1]';
Lx = width*marm_x;
Lxmat = width*diag(marm_x);
Lz = length*marm_z;
Lzmat = length*diag(marm_z);

%%%%% Water model params %%%%%%
density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
alpha = 1/16;

%Compute Taylor Series Terms%

Ay  = [Force_y(y,theta,phi,w,w,b_water,k_water,A,length,-width,alpha); 
		Force_y(y,theta,phi,w,w,b_water,k_water,A,-length,-width,alpha);
		Force_y(y,theta,phi,w,w,b_water,k_water,A,-length,width,alpha);
		Force_y(y,theta,phi,w,w,b_water,k_water,A,length,width,alpha)];

Ath  = [Force_th(y,theta,phi,w,w,b_water,k_water,A,length,-width,alpha);
		 Force_th(y,theta,phi,w,w,b_water,k_water,A,-length,-width,alpha);
		 Force_th(y,theta,phi,w,w,b_water,k_water,A,-length,width,alpha);
		 Force_th(y,theta,phi,w,w,b_water,k_water,A,length,width,alpha)];

Aph  = [Force_phi(y,theta,phi,w,w,b_water,k_water,A,length,-width,alpha);
		 Force_phi(y,theta,phi,w,w,b_water,k_water,A,-length,-width,alpha);
		 Force_phi(y,theta,phi,w,w,b_water,k_water,A,-length,width,alpha);
		 Force_phi(y,theta,phi,w,w,b_water,k_water,A,length,width,alpha)];

B1 = [Force_w1(y,theta,phi,w,w,b_water,k_water,A,length,-width,alpha);
	   Force_w1(y,theta,phi,w,w,b_water,k_water,A,-length,-width,alpha);
	   Force_w1(y,theta,phi,w,w,b_water,k_water,A,-length,width,alpha);
	   Force_w1(y,theta,phi,w,w,b_water,k_water,A,length,width,alpha)];

B2 = [Force_w2(y,theta,phi,w,w,b_water,k_water,A,length,-width,alpha);
	   Force_w2(y,theta,phi,w,w,b_water,k_water,A,-length,-width,alpha);
	   Force_w2(y,theta,phi,w,w,b_water,k_water,A,-length,width,alpha);
	   Force_w2(y,theta,phi,w,w,b_water,k_water,A,length,width,alpha)];

G =  [Force(y,theta,phi,w,w,b_water,k_water,A,length,-width,alpha); 
	  Force(y,theta,phi,w,w,b_water,k_water,A,-length,-width,alpha);
	  Force(y,theta,phi,w,w,b_water,k_water,A,-length,width,alpha);
	  Force(y,theta,phi,w,w,b_water,k_water,A,length,width,alpha)] ... 
      - Ay*y - Ath*theta - Aph*phi - B1*w - B2*w;

%Matrices for Controlller%%%
Ac = [sum(Ay), sum(Ath), sum(Aph);
	  Lx'*Ay,  Lx'*Ath, Lx'*Aph;
	  Lz'*Ay,  Lz'*Ath, Lz'*Aph];

Bc = [B1',       B2';
	  B1'*Lxmat, B2'*Lxmat;
	  B1'*Lzmat, B2'*Lzmat];
pinvBc = pinv(Bc);
nullProj = eye(8) - pinvBc*Bc;

Gc = [sum(G) - mass*g ; Lx'*G ; Lz'*G];

%Matrices for System%%%
As = [0,            1, 0,             0, 0,             0, 0;
	  sum(Ay)/mass, 0, sum(Ath)/mass, 0, sum(Aph)/mass, 0, sum(G)/mass-g;
	  0,            0, 0,             1, 0,             0, 0;
	  Lx'*Ay/Ix,    0, Lx'*Ath/Ix,    0, Lx'*Aph/Ix,    0, Lx'*G/Ix;
	  0,            0, 0,             0, 0,             1, 0;
	  Lz'*Ay/Iz,    0, Lz'*Ath/Iz,    0, Lz'*Aph/Iz,    0, Lz'*G/Iz;
	  0,            0, 0,             0, 0,             0, 0];

Bs = [zeros(1,8);
	  B1'/mass,       B2'/mass;
	  zeros(1,8);
	  B1'*Lxmat/Ix, B2'*Lxmat/Ix;
	  zeros(1,8);
	  B1'*Lzmat/Iz, B2'*Lzmat/Iz;
	  zeros(1,8)];

Cs = [1, 0, 0, 0, 0, 0, 0;
	  0, 0, 1, 0, 0, 0, 0;
	  0, 0, 0, 0, 1, 0, 0];

Ds = zeros(3,8);

%%%%% PID Gains - Ziegler Nichols Method %%%%%
Kuy = 170;
Tuy = 0.5;
Kpy = 0.2*Kuy;
Kiy = 2*Kpy/Tuy;
Kdy = Kpy*Tuy/3;

Kuth = 490;
Tuth = 0.275;
Kpth = 0.2*Kuth;
Kith = 2*Kpth/Tuth;
Kdth = Kpth*Tuth/3;

Kuph = 484;
Tuph = 0.3;
Kpph = 0.2*Kuph;
Kiph = 2*Kpph/Tuph;
Kdph = Kpph*Tuph/3;

K_p = [Kpy 0 0 ; 0 Kpth 0; 0 0 Kpph];
K_i = [Kiy 0 0; 0 Kith 0; 0 0 Kiph];
K_d = [Kdy 0 0 ; 0 Kdth 0; 0 0 Kdph];
Filter_Coef = 1000;
