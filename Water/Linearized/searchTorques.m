simParam;

torques = [2e-3, 5e-3, 8e-3, 1e-2];

if 1
	closedResponse = cell(size(torques));
	openResponse = cell(size(torques));

	i = 1;
	for dTorqueAmp = torques
		disp(dTorqueAmp)
		sim('WaterRunner_closed');
		
		time = output.time;
		[~, angles] = output.signals.values;
		closedResponse{i} = [time,angles];

		sim('WaterRunner_open');
		time = output.time;
		[~, angles] = output.signals.values;
		openResponse{i} = [time,angles];
		i = i + 1;
	end
end

figure(1)
subplot(211);
	hold all
	for i = 1:length(torques)
		plot(closedResponse{i}(:,1),closedResponse{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Closed Loop Response');
	ylabel('roll angle [degrees]');
	legend([num2str(torques'*1e3),repmat('mN-m',size(torques'))],'Location','EastOutside');
	axis([0, 10, -40, 40]);
subplot(212);
	hold all
	for i = 1:length(torques)
		plot(openResponse{i}(:,1),openResponse{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Open Loop Response');
	xlabel('time');
	ylabel('roll angle [degrees]');
	legend([num2str(torques'*1e3),repmat('mN-m',size(torques'))],'Location','EastOutside');
	axis([0, 10, -40, 40]);
