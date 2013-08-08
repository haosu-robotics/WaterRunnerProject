%%%%% Initial output %%%%%%
x0 = [0.015, 0, 0*pi/180, 0, 1]';

%%%%% Desired output %%%%%
y = 0.015;
theta = 0 *pi/180;
xdes = [y theta]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Robot Parameters %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% Leg Parameters  %%%%%%%%%%%%%%%%%%%%%%
L1 = .0615;
L2 = 0.0218;
L3 = 0.0748;
L4 = 0.0468;
L5 = 0.0624;
bar_h = 1.5e-3;
bar_w = 4.3e-3;
CF_density = 1790;

%%%%%%%%%%%%%%%%%%%%%% Body Parameters %%%%%%%%%%%%%%%%%%%%%%%%
frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

body_size = [frame_length frame_width frame_height];    % in m

FR_TR = [ frame_length/2    -frame_width/2 0];
FL_TR = [ frame_length/2    frame_width/2 0];
HR_TR = [-frame_length/2+L5 -frame_width/2 0];
HL_TR = [-frame_length/2+L5 frame_width/2 0];

frame_mass = 0.1;

%%%%%%%%%%%%%%%%%%%%%% Tail Parameters %%%%%%%%%%%%%%%%%%%%%%%%
Ta_TR = [-frame_length/2 0 0];
tail_length = 0.1;
tail_dim = [tail_length bar_w bar_h ];
tail_density = CF_density;
tail_angle = 20;
tail_pad_radius = 0.035;

%%%%%%%%%%%%%%%%%%%%%% Motor Parameters %%%%%%%%%%%%%%%%%%%%%%
motor_radius = 0.01342;
motor_length = 0.03663;

motor_mass = 0.010;
I_shaft = 1e-2;
motor_volume = (pi*motor_radius^2)*motor_length;
motor_density = motor_mass/motor_volume;

%%%%%%%%%%%%%%%%%%%%%% Mass Parameters %%%%%%%%%%%%%%%%%%%%%%
mass = frame_mass + 4*motor_mass + tail_density*prod(tail_dim) + 4*CF_density*bar_h*bar_w*(L1 + L2 +L3 + L4);
Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

marm_x = [2  -2 ]';
Lx = frame_width/2*marm_x;
Lxmat = frame_width/2*diag(marm_x);

%%%%% Water Model Parameters %%%%%

water_level = 0;
A = 0.0428;
Amp = A;

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%Controller Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% Calculate Steady-State frequncy from Desired height %%%%%
w0 = 60; %initial guess
freq = findSSinput(y,b_water,k_water,A,Fratio,mass*g/4,w0);
w = freq;
omega_0 = freq*ones(4,1);
%Cgain = 10;

%Compute Taylor Series Terms%

Ay  = [Force_y(y,theta,w,w,b_water,k_water,A,-frame_width/2,Fratio); 
	   Force_y(y,theta,w,w,b_water,k_water,A,frame_width/2,Fratio)];	

Ath  = [Force_th(y,theta,w,w,b_water,k_water,A,-frame_width/2,Fratio); 
		Force_th(y,theta,w,w,b_water,k_water,A,frame_width/2,Fratio)];	

B1 = [Force_w1(y,theta,w,w,b_water,k_water,A,-frame_width/2,Fratio);
	  Force_w1(y,theta,w,w,b_water,k_water,A,frame_width/2,Fratio)];

B2 = [Force_w2(y,theta,w,w,b_water,k_water,A,-frame_width/2,Fratio);
	  Force_w2(y,theta,w,w,b_water,k_water,A,frame_width/2,Fratio)];

G =  [Force(y,theta,w,w,b_water,k_water,A,-frame_width/2,Fratio); 
	  Force(y,theta,w,w,b_water,k_water,A,-frame_width/2,Fratio)] ...
      - Ay*y - Ath*theta - B1*w - B2*w;

%Matrices for Controlller%%%
Mc = [mass 0 ; 0 Ix]; %mass matrix for controller

Ac = [2*sum(Ay), 2*sum(Ath)
	  Lx'*Ay,  Lx'*Ath];

Bc = [2*B1',       2*B2';
	  B1'*Lxmat, B2'*Lxmat];
pinvBc = pinv(Bc);
nullProj = eye(4) - pinvBc*Bc;

Gc = [2*sum(G) - mass*g ; Lx'*G ];

%Matrices for System%
As = [0,            1, 0,             0, 0              ;
	  2*sum(Ay)/mass, 0, 2*sum(Ath)/mass, 0, 2*sum(G)/mass-g  ;
	  0,            0, 0,             1, 0              ;
	  Lx'*Ay/Ix,    0, Lx'*Ath/Ix,    0, Lx'*G/Ix       ;
	  0,            0, 0,             0, 0             ];

Bs = [zeros(1,4);
	  2*B1'/mass,       2*B2'/mass;
	  zeros(1,4);
	  B1'*Lxmat/Ix, B2'*Lxmat/Ix;
	  zeros(1,4)];

Cs = [1, 0, 0, 0, 0;
	  0, 0, 1, 0, 0];

Ds = zeros(2,4);


%%%%% PID Gains - Ziegler Nichols Method %%%%%
Kuy = 170;
%Kuy = 100;
Tuy = 0.5;
Kpy = 0.2*Kuy;
Kiy = 2*Kpy/Tuy;
Kdy = Kpy*Tuy/3;

Kuth = 490;
%Kuth = 0;
Tuth = 0.275;
Kpth = 0.2*Kuth;
Kith = 2*Kpth/Tuth;
Kdth = Kpth*Tuth/3;

K_p = [Kpy 0 ; 0 Kpth  ];
K_i = [Kiy 0 ; 0 Kith/2];
K_d = [Kdy 0 ; 0 Kdth  ];
Filter_Coef = 1000;

Kdf = 0; %duty factor preference gain
