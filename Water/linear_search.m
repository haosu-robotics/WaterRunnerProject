clc
close all
clear all

tic;
simParams;
load_system('WaterRunner.mdl');

ss = 60:1:80;
cg = 1:0.01:2.0;

[SS, CG] = meshgrid(ss,cg);

robot_speed = nan(size(SS));
robot_vert_low  = nan(size(SS));
robot_vert_high = nan(size(SS));
robot_pow = nan(size(SS));

for k = 1 : numel(SS)
	center_g = CG(k);
    frame_CG = [0 center_g 0];
	speed = SS(k);
    
	if abs(0.0227*speed  -0.3392 - center_g) > 0.05
		continue
	end
	frame_CG
	speed
	sim('WaterRunner.mdl');
    
    ind = find(position.time < position.time(end)/2);
    ind = ind(end);
	avg_h = mean(position.signals(3).values(ind:end));
	if avg_h < 0
		continue
	end   

	robot_speed(k) = mean(position.signals(2).values(ind:end));

    robot_vert_low (k) =  min(position.signals(3).values(ind:end));
    robot_vert_high(k) =  max(position.signals(3).values(ind:end));

    W_FR = work_FR.signals.values(ind:end);
    W_FL = work_FL.signals.values(ind:end);
    W_HR = work_HR.signals.values(ind:end);
    W_HL = work_HL.signals.values(ind:end);
    
    work = W_FR + W_FL + W_HR + W_HL;
    power = work(end) - work(1);
    power = power/(position.time(end)-position.time(ind));
    
    robot_pow(k) = power;
end

close_system('WaterRunner.mdl');

gravity = 9.81;
COT = robot_pow ./ (robot_speed * totalMass * gravity);

save('speed_cg.mat')
toc;
