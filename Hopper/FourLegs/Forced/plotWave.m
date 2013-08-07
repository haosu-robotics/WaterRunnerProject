if 1
	SimParam_4VDF;
	sim('water_4hopper_VDF');
	ball_position_closed = ball_position;

	sim('water_4hopper_open');
	ball_position_open = ball_position;
end

figure(1)
subplot(211) 
	plot(ball_position_closed.time,ball_position_closed.signals(1).values)
	hold on
	plot([0 30],[0.01 0.01],'b--')
	plot([0 30],[0 0],'k--')
	hold off
	ylim([-.02 .02])
	ylabel('Height [m]')

subplot(212) 
	plot(ball_position_closed.time,ball_position_closed.signals(2).values(:,1),'-b')
	hold on
	plot(ball_position_closed.time,ball_position_closed.signals(2).values(:,2),'-r')
	plot([0 30],[0 0],'b--')
	plot([0 30],[0 0],'r--')
	hold off
	legend('Roll','Pitch')
	ylabel('Angle [degrees]')
	ylim([-5 5])
	xlabel('Time [s]')

export_fig -transparent closedWave.pdf

figure(2)
subplot(211) 
	plot(ball_position_open.time,ball_position_open.signals(1).values)
	hold on
	plot([0 30],[0.01 0.01],'b--')
	plot([0 30],[0 0],'k--')
	hold off
	ylim([-.02 .02])
	ylabel('Height [m]')

subplot(212) 
	plot(ball_position_open.time,ball_position_open.signals(2).values(:,1),'-b')
	hold on
	plot(ball_position_open.time,ball_position_open.signals(2).values(:,2),'-r')
	plot([0 30],[0 0],'b--')
	plot([0 30],[0 0],'r--')
	hold off
	ylim([-5 5])
	legend('Roll','Pitch')
	ylabel('Angle [degrees]')
	xlabel('Time [s]')

export_fig -transparent openWave.pdf
