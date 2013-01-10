clc
clear all
close all

water_level = 0;
initRobHeight = 0.03;

%%%%%% frame parameters %%%%%%%%%%%%%%%%%%
frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

frame_CA = [0 0 0];
frame_FR = [ frame_length/2 0  frame_width/2 ];
frame_FL = [ frame_length/2 0 -frame_width/2 ];
frame_HR = [-frame_length/2 0  frame_width/2 ];
frame_HL = [-frame_length/2 0 -frame_width/2 ];

frame_mass = 0.060;
Ix = 1/12*frame_mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*frame_mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*frame_mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);

%%%%%%% motors parameters %%%%%%%
speed = 70;   % rotation speed in rad/s
L6 = 0.17325;    % this is distance between front and hind motors

frame_motor_FR = frame_FR;
frame_motor_FL = frame_FL;
frame_motor_HR = frame_motor_FR - [L6 0 0];
frame_motor_HL = frame_motor_FL - [L6 0 0];

motor_mass = 0.010;
motor_radius = 0.01342;
motor_length = 0.03663;

Ix = 1/12*motor_mass*(3*motor_radius^2 + motor_length^2);
Iy = Ix;
Iz = motor_mass*motor_radius^2/2;

motor_inertia = diag([Ix,Iy,Iz]);

%%%%%%%  passive joint parameters %%%%%%

L1 = .0615;   %distance between each motor and its passive joint

frame_passive_FR = frame_motor_FR - [L1 0 0];
frame_passive_FL = frame_motor_FL - [L1 0 0];
frame_passive_HR = frame_motor_HR - [L1 0 0];
frame_passive_HL = frame_motor_HL - [L1 0 0];


%%%%% leg parameters %%%%%%%%%%%%%

L2 = 0.0218;
L3 = 0.0748;
L4 = 0.0468;
L5 = 0.0624;


L2_CS1 = [0 0 0];     % using adjoining
L2_CG  = [0 L2/2 0];
L2_CS2 = [0 L2 0];

CF_density = 1790;
cross_area = 6.8e-6;

L2_mass = CF_density*cross_area*L2;
I = 1/12*L2_mass*L2^2;
L2_inertia = diag([I 0 I]);

%%%%%% calculations 

alpha =  2*L1*L4;
beta  = -2*L2*L4;
gamma = L1^2 + L4^2 + L2^2 - L3^2;

delta = atan2(beta, alpha);
theta4 = delta + acos(-gamma/(norm([alpha,beta])));
theta3  = atan2( L4*sin(theta4)-L2 , L1+L4*cos(theta4) );

%%%%%%% L4


L4_CS1 = [0 0 0];       %using adjoining
L4_CS2 = [0 L4 0];
L4_CG = L4_CS2./2;

L4_ori = [0 0 pi/2-theta4];

L4_mass = CF_density*cross_area*L4;
I = 1/12*L4_mass*L4^2;
L4_inertia = diag([I 0 I]);

%%%%


L3_CS1 = [0 0 0];       %using adjoining
L3_CS2 = [-L3 0 0];
L3_CS3 = [L5 0 0];

L3_CG = (L3_CS2 + L3_CS3)./2;

L3_ori = [0 0 -theta3];

L3_mass = CF_density*cross_area*L3;
I = 1/12*L3_mass*L3^2;
L3_inertia = diag([I 0 I]);

%%%%%%%%

totalMass = frame_mass + 4*(motor_mass + L2_mass + L3_mass + L4_mass);
