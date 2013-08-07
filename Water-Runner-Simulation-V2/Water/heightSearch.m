tic;
load_system('WaterRunner.mdl');

ss = 40:4:100;

robot_vert_avg2  = nan(size(ss));

for k = 1 : numel(ss)
	speed = ss(k)
	simParam;

	try
		sim('WaterRunner_open.mdl');
	catch
		continue
	end
   
	if sum(stopped.Data)
		continue
	end

    ind = find(position.time < position.time(end)/2);
    ind = ind(end);
	avg_h = mean(position.signals(1).values(ind:end));
	if avg_h < 0
		continue
	end

    robot_vert_avg2 (k) =  avg_h;
end

save('height.mat','robot_vert_avg2');
close_system('WaterRunner.mdl');

figure(1)
plot(ss,robot_vert_avg2);
xlabel('Rotation speed [rad/s]')
ylabel('Robot average vertical position [m]')
