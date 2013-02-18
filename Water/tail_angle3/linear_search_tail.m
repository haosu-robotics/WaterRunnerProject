clc
close all
clear all

tic;
simParams;
load_system('WaterRunner.mdl');

ss = 40:2.5:100;
ta = 0:pi/72:pi/2;

[SS, TA] = meshgrid(ss,ta);

robot_speed = nan(size(SS));
robot_vert_low  = nan(size(SS));
robot_vert_high = nan(size(SS));
robot_pow = nan(size(SS));
robot_angle = nan(size(SS));

for k = 1 : numel(SS)
	speed = SS(k)
	tail_angle = TA(k)
    
	sim('WaterRunner.mdl');
    
	if sum(stopped.Data)
		continue
	end

    ind = find(position.time < position.time(end)/2);
    ind = ind(end);
	avg_h = mean(position.signals(3).values(ind:end));

	robot_angle(k) = mean(position.signals(5).values(ind:end));

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

g = 9.81;
COT = robot_pow ./ (robot_speed * totalMass * g);

save('speed_tail.mat')
toc;
