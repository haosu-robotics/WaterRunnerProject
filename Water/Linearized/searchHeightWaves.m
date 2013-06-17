heights = [5e-3, 6e-3, 7e-3, 8e-3];

if 1
	closedResponse1 = cell(size(heights));
	closedResponse2 = cell(size(heights));
	phases1 = cell(size(heights));
	phases2 = cell(size(heights));

	i = 1;
	for dWaveAmp = heights
		clearvars -except closedResponse1 closedResponse2 phases1 phases2 i heights dWaveAmp
		simParam;

		dWaveAmp
		Cgain = 10
		sim('WaterRunner_closed3');
		
		time = output.time;
		[~, angles] = output.signals.values;
		closedResponse1{i} = [time,angles];
		phase = cpg_omega.signals(2).values(:,4); 
		phases1{i} = [time,phase];
		
		clearvars -except closedResponse1 closedResponse2 phases1 phases2 i heights dWaveAmp
		simParam;

		dWaveAmp
		Cgain = 0
		sim('WaterRunner_closed3');
		time = output.time;
		[~, angles] = output.signals.values;
		closedResponse2{i} = [time,angles];
		phase = cpg_omega.signals(2).values(:,4); 
		phases2{i} = [time,phase];
		i = i + 1;
	end
end

save('data1.mat','closedResponse1','phases1','closedResponse2','phases2')
figure(1)
subplot(211);
	hold all
	for i = 1:length(heights)
		plot(closedResponse1{i}(:,1),closedResponse1{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Response w/ heuristic');
	ylabel('roll angle [degrees]');
	legend([num2str(heights'*1e3),repmat('mm',size(heights'))],'Location','EastOutside');
	axis([0, 10, -45 45]);
subplot(212);
	hold all
	for i = 1:length(heights)
		plot(phases1{i}(:,1),phases1{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Leg Phase Relationship');
	xlabel('time');
	ylabel('phase [radians]');
	legend([num2str(heights'*1e3),repmat('mm',size(heights'))],'Location','EastOutside');
	axis([0, 10, 0, 2*pi]);


figure(2)
subplot(211);
	hold all
	for i = 1:length(heights)
		plot(closedResponse2{i}(:,1),closedResponse2{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Response w/o heuristic');
	ylabel('roll angle [degrees]');
	legend([num2str(heights'*1e3),repmat('mN-m',size(heights'))],'Location','EastOutside');
	axis([0, 10, -45 45]);
subplot(212);
	hold all
	for i = 1:length(heights)
		plot(phases2{i}(:,1),phases2{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Leg Phase Relationship');
	xlabel('time');
	ylabel('phase [radians]');
	legend([num2str(heights'*1e3),repmat('mN-m',size(heights'))],'Location','EastOutside');
	axis([0, 10, 0, 2*pi]);
