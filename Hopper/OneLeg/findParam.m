SimParam;

forceRatio = (r2/r1)^2;

load_system('water_hopper.mdl');
T_des = 20;

freqs = 40:4:100;

y_pred = zeros(size(freqs));

robot_vert_avg =  open('heightV2.mat');
rheights3 = robot_vert_avg.robot_vert_avg;
rheights3(isnan(rheights3)) = 0;

coeff = nlinfit(freqs,rheights3,@SSheight3,[.04,.7])
