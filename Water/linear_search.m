clc
close all
clear all

tic;
simParams;
load_system('WaterRunner.mdl');

SS = 40:5:80;
DF = .20:.10:.80;

robot_speed = nan(size(SS),size(DF));
robot_vert_low  = nan(size(SS),size(DF));
robot_vert_high = nan(size(SS),size(DF));
robot_pow = nan(size(SS),size(DF));

for j = 1 : numel(SS)
	speed = SS(j);
	for k = numel(DF)
		duty_factor = DF(k)
        DF_param
		sim('WaterRunner.mdl');
		
		ind = find(position.time < position.time(end)/2);
		ind = ind(end);
		avg_h = mean(position.signals(3).values(ind:end));
		if avg_h < 0
			continue
		end   
		robot_vert_avg(j,k) = avg_h;
		robot_vert_low (j,k) =  min(position.signals(3).values(ind:end));
		robot_vert_high(j,k) =  max(position.signals(3).values(ind:end));
		
		robot_speed(j,k) = mean(position.signals(2).values(ind:end));

		W_FR = work_FR.signals.values(ind:end);
		W_FL = work_FL.signals.values(ind:end);
		W_HR = work_HR.signals.values(ind:end);
		W_HL = work_HL.signals.values(ind:end);
		
		work = W_FR + W_FL + W_HR + W_HL;
		power = work(end) - work(1);
		power = power/(position.time(end)-position.time(ind));
		
		robot_pow(j,k) = power;
end

close_system('WaterRunner.mdl');

gravity = 9.81;
COT = robot_pow ./ (robot_speed * totalMass * gravity);

save('speed_df.mat')
toc;
