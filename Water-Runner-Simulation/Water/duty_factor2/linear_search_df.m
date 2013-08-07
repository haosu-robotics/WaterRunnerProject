clc
close all
clear all

tic;
load_system('WaterRunner.mdl');

ss = 20:5:100;
df = .2:.05:.8;

[SS, DF] = meshgrid(ss,df);

robot_speed = nan(size(SS));
robot_vert_low  = nan(size(SS));
robot_vert_high = nan(size(SS));
robot_pow = nan(size(SS));
robot_angle = nan(size(SS));

for k = 1 : numel(SS)
	speed = SS(k)
	duty_factor = DF(k)
    simParams

	sim('WaterRunner.mdl');
    
	if sum(stopped.Data)
		continue
	end

    ind = find(position.time < position.time(end)/2);
    ind = ind(end);
	avg_h = mean(position.signals(3).values(ind:end));
	if avg_h < 0
		continue
	end

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

save('speed_df.mat')
toc;
