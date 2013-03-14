clc
close all
clear all

%%%%%%%%%%%%%
mass = 0.1;
m_toe = 0.000001;
leg_length = 0.06;

frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

TR_FR = [ frame_length/2 0  frame_width/2 ];
TR_FL = [ frame_length/2 0 -frame_width/2 ];
TR_HR = [-frame_length/2 0  frame_width/2 ];
TR_HL = [-frame_length/2 0 -frame_width/2 ];

Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);


%%%%%%%%%%%%%


amp = leg_length/4;
freq = 20;

amp_group     = amp*freq.^[0 1 2];
phase_group_1 = 0  + [0 pi/2 pi];
phase_group_2 = pi + [0 pi/2 pi];

y_0 = 0.12;

r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2;
S2 = pi*r2^2;

density = 1000;
g = 9.81;
C_d = 0.703;



