water_level = 0;
initRobHeight = 0.03;

%%%%%%% motors parameters %%%%%%%
speed = 80;   % rotation speed in rad/s
duty_factor = .5;
switch_angle = 0;

V_H = speed/(2*duty_factor);
V_L = speed/(2*(1 - duty_factor));

pulse_df = duty_factor*100;
pulse_amplitude = V_H - V_L;
pulse_period = 2*pi/speed;
median_speed = (V_H + V_L)/2 - pulse_amplitude/2;

phase_delay = switch_angle/V_L;
tau = 2*pi/speed/200;

motor_mass = 0.010;
motor_radius = 0.01342;
motor_length = 0.03663;

Ix = 1/12*motor_mass*(3*motor_radius^2 + motor_length^2);
Iy = Ix;
Iz = motor_mass*motor_radius^2/2;

motor_inertia = diag([Ix,Iy,Iz]);

%%%%%% frame parameters %%%%%%%%%%%%%%%%%%
frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

frame_CA = [0 0 0];
frame_CG = [0,0,0];
frame_FR = [ frame_length/2 0  frame_width/2 ];
frame_FL = [ frame_length/2 0 -frame_width/2 ];
frame_HR = [-frame_length/2 0  frame_width/2 ];
frame_HL = [-frame_length/2 0 -frame_width/2 ];

frame_mass = .140-4*motor_mass;
Ix = 1/12*frame_mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*frame_mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*frame_mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

L6 = 0.17325;    % this is distance between front and hind motors

frame_motor_FR = frame_FR;
frame_motor_FL = frame_FL;
frame_motor_HR = frame_motor_FR - [L6 0 0];
frame_motor_HL = frame_motor_FL - [L6 0 0];
frame_motor_Tail = (frame_HR + frame_HL)/2;


%%%%%%%  passive joint parameters %%%%%%

L1 = .0615;   %distance between each motor and its passive joint

frame_passive_FR = frame_motor_FR - [L1 0 0];
frame_passive_FL = frame_motor_FL - [L1 0 0];
frame_passive_HR = frame_motor_HR - [L1 0 0];
frame_passive_HL = frame_motor_HL - [L1 0 0];


%%%%% leg parameters %%%%%%%%%%%%%
CF_density = 1790;
cross_area = 6.8e-6;

L2 = 0.0218;
L3 = 0.0748;
L4 = 0.0468;
L5 = 0.0624;


L2_CS1 = [0 0 0];     % using adjoining
L2_CG  = [0 L2/2 0];
L2_CS2 = [0 L2 0];


L2_mass = CF_density*cross_area*L2;
I = 1/12*L2_mass*L2^2;
L2_inertia = diag([I 0 I]);

%%%%%% calculations %%%%%%

alpha =  2*L1*L4;
beta  = -2*L2*L4;
gamma = L1^2 + L4^2 + L2^2 - L3^2;

delta = atan2(beta, alpha);
theta4 = delta + acos(-gamma/(norm([alpha,beta])));
theta3  = atan2( L4*sin(theta4)-L2 , L1+L4*cos(theta4) );

%%%%%%% L4 %%%%%%


L4_CS1 = [0 0 0];       %using adjoining
L4_CS2 = [0 L4 0];
L4_CG = L4_CS2./2;

L4_ori = [0 0 pi/2-theta4];

L4_mass = CF_density*cross_area*L4;
I = 1/12*L4_mass*L4^2;
L4_inertia = diag([I 0 I]);

%%%%%% L3 %%%%%%

L3_CS1 = [0 0 0];       %using adjoining
L3_CS2 = [-L3 0 0];
L3_CS3 = [L5 0 0];

L3_CG = (L3_CS2 + L3_CS3)./2;

L3_ori = [0 0 -theta3];

L3_mass = CF_density*cross_area*L3;
I = 1/12*L3_mass*L3^2;
L3_inertia = diag([I 0 I]);

%%%%%%%% Tail Parameters %%%%%%%

tail_angle = 15 * pi/180;
L_tail = .1;
Tail_CS1 = [0 0 0];     % using adjoining
Tail_CG  = [0 L_tail/2 0];
Tail_CS2 = [0 L_tail 0];

Tail_mass = CF_density*cross_area*L_tail;
I = 1/12*Tail_mass*L_tail^2;
Tail_inertia = diag([I 0 I]);

%%%%%
mass = frame_mass + 4*(motor_mass + L2_mass + L3_mass + L4_mass) + motor_mass + Tail_mass
Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

mass_matrix = [mass 0 0; 0 Ix 0 ; 0 0 Iz];

Lx = frame_width/2*[1 1 -1 -1]';
Lxmat = frame_width/2*[1 0 0 0 ; 0 1 0 0 ; 0 0 -1 0 ; 0 0 0 -1];
Lz = frame_length/2*[1 -1 -1 1]';
Lzmat = frame_length/2*[1 0 0 0 ; 0 -1 0 0 ; 0 0 -1 0 ; 0 0 0 1];

%%%%% Water Model params %%%%%

leg_length = 0.0249;
Amp = leg_length;
amp = Amp;
freq = 60;
omega_0 = freq*ones(8,1);
omega_1 = freq*ones(4,1);

y_0 = .01;

r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2*0.9185;
S2 = pi*r2^2*0.9185;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

waveAmp = 0.01;
rand('seed',1);
wavePhase = 2*pi*rand(1,4);
waveFreq = 0.5;

%%% PID gains %%%%
K_p = [500 0 0 ; 0 750 0; 0 0 750];
K_i = [800 0 0 ; 0 7500 0; 0 0 7500];
K_d = [0 0  0; 0 300 0; 0 0 300];
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

tau = 0.005;
alpha = 30;
t_fil =0.05;
