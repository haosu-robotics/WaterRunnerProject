clear

%%%%% body params %%%%%
mass = 0.1;
m_toe = eps;
leg_length = 0.0249;

frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

CG_pos =[frame_length*0.0 0 0];

TR_FR = [ frame_length/2 0  frame_width/2 ];
TR_FL = [ frame_length/2 0 -frame_width/2 ];
TR_HR = [-frame_length/2 0  frame_width/2 ];
TR_HL = [-frame_length/2 0 -frame_width/2 ];

Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

mass_matrix = [mass 0 0; 0 Ix 0 ; 0 0 Iz];
Lx = frame_length/2*[-1 -1 1 1]';
Lxmat = frame_length/2*[-1 0 0 0 ; 0 -1 0 0 ; 0 0 1 0 ; 0 0 0 1];
Lz = frame_width/2*[-1 1 1 -1]';
Lzmat = frame_width/2*[-1 0 0 0 ; 0 1 0 0 ; 0 0 1 0 ; 0 0 0 -1];

%%%%% Water Model params %%%%%

Amp = leg_length;
amp = Amp;
freq = 60;
omega_0 = freq*ones(8,1);
omega_1 = freq*ones(2,1);

y_0 = leg_length/2;

r1 = 0.020;
r2 = r1/4;
S1 = pi*r1^2*0.9185;
S2 = pi*r2^2*0.9185;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

wave = .025;
wavefreq = .5;

%%% PID gains %%%%
K_p = [500 0 0 ; 0 1000 0; 0 0 1000];
K_i = [1000 0 0 ; 0 10000 0; 0  0 10000];
K_d = [50 0 0; 0 600 0; 0 0 600];
Filter_Coef = 1000;

%%%%% CPG params %%%%%
FR_FL_lag = pi;     % l1
HR_HL_lag = -pi;    % l2
FR_HR_lag = pi;     % l3
FL_HL_lag = -pi;    % l4

c_gain = 100;
cw_FR_FL = c_gain;
cw_HR_HL = c_gain;
cw_FR_HR = c_gain;
cw_FL_HL = c_gain;

tau = 0.001;
alpha = 30;
t_fil =0.05;
