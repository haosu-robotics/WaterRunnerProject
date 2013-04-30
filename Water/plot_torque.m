sim('WaterRunner_closed');
time = position.time;
[height, angles] = position.signals.values;

figure(1)
subplot(211);
	plot(time,height,'LineWidth',2);
	ylabel('height [m]');
	axis([0, 15, -0.015, 0.035]);
subplot(212);
	plot(time,angles(:,1),'b','LineWidth',2);
	hold on
	plot(time,angles(:,2),'r','LineWidth',2);
	hold off 
	ylabel('angle [degrees]');
	xlabel('time');
	axis([0, 15, -15, 15]);
	legend('roll','pitch','Location','NorthEast')

sim('WaterRunner_closed2');
time = position.time;
[height, angles] = position.signals.values;

subplot(211);
	hold on
	plot(time,height,'--');
	ylabel('height [m]');
	axis([0, 15, -0.015, 0.035]);
subplot(212);
	hold on
	plot(time,angles(:,1),'b--');
	plot(time,angles(:,2),'r--');
	hold off 
	ylabel('angle [degrees]');
	xlabel('time');
	axis([0, 15, -15, 15]);
	legend('roll','pitch','Location','NorthWest')
