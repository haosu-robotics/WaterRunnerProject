simParam;

tangles = 10:5:30;

if 1
	closedResponse = cell(size(tangles));
	openResponse = cell(size(tangles));

	i = 1;
	for tail_angle = tangles
		disp(tail_angle)
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
subplot(221);
	hold all
	for i = 1:length(tangles)
		plot(closedResponse{i}(:,1),closedResponse{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Closed Loop Response');
	ylabel('roll angle [degrees]');
	legend(num2str(tangles'));
	axis([0, 10, -40, 40]);
subplot(222);
	hold all
	for i = 1:length(tangles)
		plot(openResponse{i}(:,1),openResponse{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Open Loop Response');
	ylabel('roll angle [degrees]');
	legend(num2str(tangles'));
	axis([0, 10, -40, 40]);
subplot(223);
	hold all
	for i = 1:length(tangles)
		plot(closedResponse{i}(:,1),closedResponse{i}(:,3),'LineWidth',2);
	end
	hold off
	xlabel('time');
	ylabel('pitch angle [degrees]');
	legend(num2str(tangles'));
	axis([0, 10, -40, 40]);
subplot(224);
	hold all
	for i = 1:length(tangles)
		plot(openResponse{i}(:,1),openResponse{i}(:,3),'LineWidth',2);
	end
	hold off
	ylabel('pitch angle [degrees]');
	xlabel('time');
	legend(num2str(tangles'));
	axis([0, 10, -40, 40]);
