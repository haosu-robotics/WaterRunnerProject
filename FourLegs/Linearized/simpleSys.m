%%%%% Initial State %%%%%%
y0 = 0.015;
roll0 = 0;
pitch0 = 0;
x0 = [y0; 0; roll0; 0; pitch0; 0; 1];

%%%%% Desired State %%%%%%
y = 0.015;
roll = 5;
pitch = 0;
w = 60;
xdes = [y, roll, pitch];

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

frame_width  = 0.125; 
frame_length = 0.25;
M = [mass 0 0; 0 Ix 0 ; 0 0 Iz]; %mass matrix
  
marm_x = [1 1 -1 -1]';
marm_y = [1 -1 -1 1]';
Lx = frame_width/2*marm_x;
Lxmat = frame_width/2*diag(marm_x);
Lz = frame_length/2*marm_y;
Lzmat = frame_length/2*diag(marm_y);

%%%%% Water model params %%%%%%
density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
alpha = 1/16;

%Compute Taylor Series Terms%

Ai  = Force_y(y,w,w,b_water,k_water,A,alpha);
Bi1 = Force_w1(y,w,w,b_water,k_water,A,alpha);
Bi2 = Force_w2(y,w,w,b_water,k_water,A,alpha);

Gi = Force(y,w,w,b_water,k_water,A,alpha) - Ai*y - Bi1*w - Bi2*w;

%Matrices for Controlller%%%
Ac = [4*Ai, 0, 0;
		0,	  0, 0;
		0,    0, 0];

Bc = [Bi1*ones(1,4),       Bi2*ones(1,4);
	  Bi1*ones(1,4)*Lxmat, Bi2*ones(1,4)*Lxmat;
	  Bi1*ones(1,4)*Lzmat, Bi2*ones(1,4)*Lzmat];
pinvBc = pinv(Bc);
nullProj = eye(8) - pinvBc*Bc;

Gc = [4*Gi - mass*g ; 0; 0];

%Matrices for System%%%
As = [0,         1, 0, 0, 0, 0, 0;
	  4*Ai/mass, 0, 0, 0, 0, 0, 4*Gi/mass-g;
	  0,         0, 0, 1, 0, 0, 0;
	  0,         0, 0, 0, 0, 0, 0;
	  0,         0, 0, 0, 0, 1, 0;
	  0,         0, 0, 0, 0, 0, 0;
	  0,         0, 0, 0, 0, 0, 0];

Bs = [zeros(1,8);
	  Bi1*ones(1,4)/mass,       Bi2*ones(1,4)/mass;
	  zeros(1,8);
	  Bi1*ones(1,4)*Lxmat/Ix, Bi2*ones(1,4)*Lxmat/Ix;
	  zeros(1,8);
	  Bi1*ones(1,4)*Lzmat/Iz, Bi2*ones(1,4)*Lzmat/Iz;
	  zeros(1,8)];

Cs = [1, 0, 0, 0, 0, 0, 0;
	  0, 0, 1, 0, 0, 0, 0;
	  0, 0, 0, 0, 1, 0, 0];

Ds = zeros(3,8);

%%%%% PID Gains - Ziegler Nichols Method %%%%%
Kuy = 160;
Tuy = 0.5;
Kpy = 0.2*Kuy;
Kiy = 2*Kpy/Tuy;
Kdy = Kpy*Tuy/3;

KuRoll = 0.002;
TuRoll = 200;
KpRoll = 0.6*KuRoll;
KiRoll = 2*KpRoll/TuRoll;
KdRoll = KpRoll*TuRoll/8;

K_p = [Kpy 0 0 ; 0 KpRoll 0; 0 0 0];
K_i = [Kiy 0 0 ; 0 KiRoll 0; 0 0 0];
K_d = [Kdy 0 0 ; 0 KdRoll 0; 0 0 0];
Filter_Coef = 1000;




