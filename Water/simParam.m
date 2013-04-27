clear all

%%%%%%%%%%%%%%%%%%%%%%% Leg properties %%%%%%%%%%%%%%%%%%%%
L1 = .0615;
L2 = 0.0218;
L3 = 0.0748;
L4 = 0.0468;
L5 = 0.0624;
bar_h = 1.5e-3;
bar_w = 4.3e-3;
CF_density = 1790;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

I_shaft = 1e-2;
%%%%%%%% tail
Ta_TR = [-frame_length/2   0             0];
tail_length = 0.1;
tail_dim = [tail_length bar_w bar_h ];
tail_density = CF_density;
tail_angle = 20;
tail_pad_radius = 0.03;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Motor Parameters %%%%%%%%%%%%%%%%%%%%%%
%speed = 70;

motor_radius = 0.01342;
motor_length = 0.03663;

motor_mass = 0.010;
motor_volume = (pi*motor_radius^2)*motor_length;
motor_density = motor_mass/motor_volume;

mass = frame_mass + 4*motor_mass + tail_density*prod(tail_dim) + 4*CF_density*bar_h*bar_w*(L1 + L2 +L3 + L4)
Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

mass_matrix = [mass 0 0; 0 Ix 0 ; 0 0 Iz];


%%%%%%%%%%%%%%%%%% Ground Contact Model %%%%%%%%%%%%%%%%%%%%%%%

k_gy = 8e4; %[N/m]  vertical ground interaction stiffness
v_gy_max = 0.03; %[m/s] maximum vertical ground relaxation speed
k_gx = 8e3; %[N/m] horizontal ground interaction stiffness
v_gx_max = 0.03; %[m/s] maximum horizontal ground relaxation speed
mu_stick = 0.9; %stiction coefficient
mu_slide = 0.8; % sliding coefficient
vLimit = 0.01; %[m/s] % slip-stic transition velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PVA_gains = [0.0 0.05 0.00];

%%%%% Water Model Parameters %%%%%

water_level = 0;
leg_length = 0.0249;
Amp = leg_length;
amp = Amp;

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

%%% PID gains %%%%
K_p = [500 0 0 ; 0 750 0; 0 0 750];
K_i = [800 0 0 ; 0 7500 0; 0 0 7500];
K_d = [0 0  0; 0 300 0; 0 0 300];
Filter_Coef = 1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
freq = 80;
speed = freq;
omega_0 = freq*ones(8,1);

FR_FL_lag = -pi;     % l1
HR_HL_lag = pi;    % l2
FR_HR_lag = -pi;     % l3
FL_HL_lag = pi;    % l4

c_gain = 100;
cw_FR_FL = c_gain;
cw_HR_HL = c_gain;
cw_FR_HR = c_gain;
cw_FL_HL = c_gain;

tau = 0.005;
alpha = 30;
t_fil =0.05;
