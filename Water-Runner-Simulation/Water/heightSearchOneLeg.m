tic;
load_system('WaterRunner.mdl');

ss = 12:8:100;
dfs = .5;
[SS, DF] = meshgrid(ss,dfs);

robot_speed = nan(size(SS));
robot_vert_avg  = nan(size(SS));
robot_pow = nan(size(SS));
robot_angle = nan(size(SS));

for k = 1 : numel(SS)
	speed = SS(k)
	duty_factor  = DF(k)
	simParams

	try
		sim('WaterRunner.mdl');
	catch
		continue
	end
   
	if sum(stopped.Data)
		continue
	end

    ind = find(position.time < position.time(end)/2);
    ind = ind(end);
	avg_h = mean(position.signals(3).values(ind:end));
	if avg_h < 0
		continue
	end

	robot_speed(k) = mean(position.signals(2).values(ind:end));
    robot_vert_avg (k) =  mean(position.signals(3).values(ind:end));

    W_FR = work_FR.signals.values(ind:end);
    
    work = W_FR;
    power = work(end) - work(1);
    power = power/(position.time(end)-position.time(ind));
    
    robot_pow(k) = power;
end

save('height3.mat','robot_vert_avg');
close_system('WaterRunner.mdl');

figure(1)
plot(SS,robot_vert_avg);
xlabel('Rotation speed [rad/s]')
ylabel('Robot average vertical position [m]')
set(gca, 'Color', 'None')

%{
figure(2)
surf(SS,DF,robot_pow)
xlabel('Rotation speed [rad/s]')
ylabel('Power Consumption')
set(gca, 'Color', 'None')
%}
