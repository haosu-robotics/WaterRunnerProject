%%%%% Initial State %%%%%%
y0 = 0.020;
theta0 = 0;
phi0 = 0;
x0 = [y0; 0; theta0; 0; phi0; 0; 1];

%%%%% Desired State %%%%%
y = 0.02;
theta = 5 *pi/180;
phi = 0 *pi/180;

%%%% disturbance torque waves %%%%%
dTorqueAmp = 0;
rand('seed',1);
dTorquePhase = 2*pi*rand(1,4);
dTorqueFreq = 0.5;

%%%% unobserved height waves %%%%%
waveAmp = 0;
rand('seed',1);
wavePhase = 2*pi*rand(1,4);
waveFreq = 0.5;

%%%%% body params %%%%%
mass = 0.15;
m_toe = eps;
leg_length = 0.0428;

frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

CG_pos =[frame_length*0 0 0];

TR_FR = [ frame_length/2 0  frame_width/2 ];
TR_FL = [ frame_length/2 0 -frame_width/2 ];
TR_HR = [-frame_length/2 0  frame_width/2 ];
TR_HL = [-frame_length/2 0 -frame_width/2 ];

Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

mass_matrix = [mass 0 0; 0 Ix 0 ; 0 0 Iz];
marm_x = [1 1 -1 -1]';
marm_z = [1 -1 -1 1]';
Lx = frame_width/2*marm_x;
Lxmat = frame_width/2*diag(marm_x);
Lz = frame_length/2*marm_z;
Lzmat = frame_length/2*diag(marm_z);

%%%%% Water Model params %%%%%

A = leg_length;
Amp = A;
amp = A;

r1 = 0.02;
r2 = r1/4;
area = 0.5449;
S1 = pi*r1^2*area;
S2 = pi*r2^2*area;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

%%%%% Calculate Steady-State frequncy from Desired height %%%%%
w0 = 60; %initial guess
freq = findSSinput(y,b_water,k_water,A,Fratio,mass*g/4,w0);
w = freq;
omega_0 = freq*ones(8,1);
omega_1 = freq*ones(4,1);


%Compute Taylor Series Terms%

Ay  = [Force_y(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,-frame_width/2,Fratio); 
		Force_y(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,-frame_width/2,Fratio);
		Force_y(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,frame_width/2,Fratio);
		Force_y(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,frame_width/2,Fratio)];

Ath  = [Force_th(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,-frame_width/2,Fratio);
		 Force_th(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,-frame_width/2,Fratio);
		 Force_th(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,frame_width/2,Fratio);
		 Force_th(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,frame_width/2,Fratio)];

Aph  = [Force_phi(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,-frame_width/2,Fratio);
		 Force_phi(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,-frame_width/2,Fratio);
		 Force_phi(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,frame_width/2,Fratio);
		 Force_phi(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,frame_width/2,Fratio)];

B1 = [Force_w1(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,-frame_width/2,Fratio);
	   Force_w1(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,-frame_width/2,Fratio);
	   Force_w1(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,frame_width/2,Fratio);
	   Force_w1(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,frame_width/2,Fratio)];

B2 = [Force_w2(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,-frame_width/2,Fratio);
	   Force_w2(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,-frame_width/2,Fratio);
	   Force_w2(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,frame_width/2,Fratio);
	   Force_w2(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,frame_width/2,Fratio)];

G =  [Force(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,-frame_width/2,Fratio); 
	  Force(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,-frame_width/2,Fratio);
	  Force(y,theta,phi,w,w,b_water,k_water,A,-frame_length/2,frame_width/2,Fratio);
	  Force(y,theta,phi,w,w,b_water,k_water,A,frame_length/2,frame_width/2,Fratio)] ... 
      - Ay*y - Ath*theta - Aph*phi - B1*w - B2*w;

%Matrices for Controlller%%%
Mc = [mass 0 0; 0 Ix 0 ; 0 0 Iz]; %mass matrix for controller

Ac = [sum(Ay), sum(Ath), sum(Aph);
	  Lx'*Ay,  Lx'*Ath, Lx'*Aph;
	  Lz'*Ay,  Lz'*Ath, Lz'*Aph];

Bc = [B1',       B2';
	  B1'*Lxmat, B2'*Lxmat;
	  B1'*Lzmat, B2'*Lzmat];
pinvBc = pinv(Bc);
nullProj = eye(8) - pinvBc*Bc;

Gc = [sum(G) - mass*g ; Lx'*G ; Lz'*G];

%%%%% PID Gains - Ziegler Nichols Method %%%%%
Kuy = 170;
Tuy = 0.5;
Kpy = 0.6*Kuy;
Kiy = 2*Kpy/Tuy;
Kdy = Kpy*Tuy/8;

Kuth = 490;
Tuth = 0.275;
Kpth = 0.6*Kuth;
Kith = 2*Kpth/Tuth;
Kdth = Kpth*Tuth/8;

Kuph = 484;
Tuph = 0.3;
Kpph = 0.6*Kuph;
Kiph = 2*Kpph/Tuph;
Kdph = Kpph*Tuph/8;

K_p = [Kpy 0 0 ; 0 Kpth 0; 0 0 Kpph];
K_i = [Kiy 0 0; 0 Kith 0; 0 0 Kiph];
K_d = [Kdy 0 0 ; 0 Kdth 0; 0 0 Kdph];
Filter_Coef = 1000;

Kdf = 0.25; %duty factor preference gain

%%%%% CPG params %%%%%
FR_FL_lag = pi;     % l1
HR_HL_lag = -pi;    % l2
FR_HR_lag = pi;     % l3
FL_HL_lag = -pi;    % l4

c_gain = 35;
cw_FR_FL = c_gain;
cw_HR_HL = c_gain;
cw_FR_HR = c_gain;
cw_FL_HL = c_gain;

tau = 0.001;
alpha = 30;
t_fil =0.05;
