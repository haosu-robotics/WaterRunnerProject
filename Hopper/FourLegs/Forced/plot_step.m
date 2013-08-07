if 1
	SimParam_4VDF;
	stepSizeRoll = 5;
	stepSizePitch = 0;
	sim('water_4hopper_VDF');
	ball_position5r = ball_position;

	stepSizeRoll = 0;
	stepSizePitch = 3;
	sim('water_4hopper_VDF');
	ball_position3p = ball_position;

	stepSizeRoll = 10;
	stepSizePitch = 0;
	sim('water_4hopper_VDF');
	ball_position10r = ball_position;
end

figure(1)
subplot(211) 
	plot(ball_position5r.time,ball_position5r.signals(1).values)
	hold on
	plot([0 20],[0.01 0.01],'b--')
	hold off
	ylabel('Height [m]')

subplot(212) 
	plot(ball_position5r.time,ball_position5r.signals(2).values(:,1),'-b')
	hold on
	plot(ball_position5r.time,ball_position5r.signals(2).values(:,2),'-r')
	plot([0 5 5 10 10  15 15 20],[0 0 5 5 -5 -5 0 0],'b--')
	plot([0 20],[0 0],'r--')
	hold off
	legend('Roll','Pitch')
	ylabel('Angle [degrees]')
	xlabel('Time [s]')

export_fig -transparent roll5.pdf

figure(2)
subplot(211) 
	plot(ball_position3p.time,ball_position3p.signals(1).values)
	hold on
	plot([0 20],[0.01 0.01],'b--')
	hold off
	ylabel('Height [m]')

subplot(212) 
	plot(ball_position3p.time,ball_position3p.signals(2).values(:,1),'-b')
	hold on
	plot(ball_position3p.time,ball_position3p.signals(2).values(:,2),'-r')
	plot([0 20],[0 0],'b--')
	plot([0 5 5 10 10  15 15 20],[0 0 3 3 -3 -3 0 0],'r--')
	hold off

	legend('Roll','Pitch')
	ylabel('Angle [degrees]')
	xlabel('Time [s]')

export_fig -transparent pitch3.pdf

figure(3)
subplot(211) 
	plot(ball_position10r.time,ball_position10r.signals(1).values)
	hold on
	plot([0 20],[0.01 0.01],'b--')
	hold off
	ylabel('Height [m]')


subplot(212) 
	plot(ball_position10r.time,ball_position10r.signals(2).values(:,1),'-b')
	hold on
	plot(ball_position10r.time,ball_position10r.signals(2).values(:,2),'-r')
	plot([0 5 5 10 10  15 15 20],[0 0 10 10 -10 -10 0 0],'b--')
	plot([0 20],[0 0],'r--')
	hold off
	legend('Roll','Pitch')
	ylabel('Angle [degrees]')
	xlabel('Time [s]')
	ylim([-12 12])
export_fig -transparent roll10.pdf

figure(4)
plot(command_omega.time,command_omega.signals.values)
title('Velocity Requested by Controller')
ylabel('Leg Angular Velocity [rad/s]')
xlabel('Time [s]')
export_fig -transparent rotationSpeed1.pdf

figure(5)
subplot(211)
	plot(command_omega.time,command_omega.signals.values)
	title('Velocity Requested by Controller')
	ylabel('Leg Angular Velocity [rad/s]')
	xlabel('Time [s]')
subplot(212)
	plot(CPG_omega.time,CPG_omega.signals.values)
	title('Velocity Provided by CPG')
	ylabel('Leg Angular Velocity [rad/s]')
	xlabel('Time [s]')

export_fig -transparent rotationSpeed2.pdf
