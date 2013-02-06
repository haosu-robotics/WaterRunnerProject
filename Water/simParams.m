clear all

%%%%%%%%%%%%%%%%%%%%%%% one leg properties %%%%%%%%%%%%%%%%%%%%
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

body_size = [frame_length frame_height frame_width];    % in m

FR_TR = [ frame_length/2    0  frame_width/2 ];
FL_TR = [ frame_length/2    0 -frame_width/2 ];
HR_TR = [-frame_length/2+L5 0  frame_width/2 ];
HL_TR = [-frame_length/2+L5 0 -frame_width/2 ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Motor Parameters %%%%%%%%%%%%%%%%%%%%%%
motor_mass = 0.010;
motor_radius = 0.01342;
motor_length = 0.03663;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

frame_mass = 0.060;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

k_gy = 8e4; %[N/m]  vertical ground interaction stiffness
v_gy_max = 0.03; %[m/s] maximum vertical ground relaxation speed
k_gx = 8e3; %[N/m] horizontal ground interaction stiffness
v_gx_max = 0.03; %[m/s] maximum horizontal ground relaxation speed
mu_stick = 0.9; %stiction coefficient
mu_slide = 0.8; % sliding coefficient
vLimit = 0.01; %[m/s] % slip-stic transition velocity

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
speed = 70;

tau = 2*pi/speed/200;

motor_radius = 0.01342;
motor_length = 0.03663;

motor_mass = 0.010;
motor_volume = (pi*motor_radius^2)*motor_length;
motor_density = motor_mass/motor_volume;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

PVA_gains = [0.0 0.05 0.00];

%%%%% Water Model Parameters %%%%%

water_level = 0;
