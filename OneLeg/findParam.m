SimParam;

forceRatio = (r2/r1)^2;

load_system('water_hopper.mdl');
T_des = 20;

freqs = 12:8:100;
amps = 0.02:0.0025:0.035;
areas = 0.5:0.025:1;

[FREQ, AMPS, AREA] = meshgrid(freqs,amps,areas);

y_pred = zeros(size(FREQ));

robot_vert_avg =  open('height3.mat');
rheights3 = robot_vert_avg.robot_vert_avg;
rheights3(isnan(rheights3)) = 0;

coeff = nlinfit(freqs,rheights3,@SSheight3,[.025,.9])
