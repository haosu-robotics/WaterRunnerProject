clc
close all
clear all

%%%%%%%%%%%%%
mass = 0.1;
m_toe = 0.0;
leg_length = 0.0249;

frame_width  = 0.125; 
frame_length = 0.25;
frame_height = 0.02;

CG_pos =[frame_length*0.1 0 0];

TR_FR = [ frame_length/2 0  frame_width/2 ];
TR_FL = [ frame_length/2 0 -frame_width/2 ];
TR_HR = [-frame_length/2 0  frame_width/2 ];
TR_HL = [-frame_length/2 0 -frame_width/2 ];

Ix = 1/12*mass*( frame_width^2 +  frame_height^2 );
Iy = 1/12*mass*( frame_width^2 +  frame_length^2 );
Iz = 1/12*mass*( frame_length^2 + frame_height^2 );

frame_inertia = diag([Ix,Iy,Iz]);


%%%%%%%%%%%%%



% duty_factor = 0.3;
amp = leg_length;
freq = 60;

y_0 = leg_length;
y_0 = 0.01;

r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2*0.9185;
S2 = pi*r2^2*0.9185;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
FRatio = 1/16;


tau = 0.001;
alpha = 30;
t_fil =0.05;
%%
K_p = -50;
K_i = 0;
K_d = 0;

