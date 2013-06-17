simParam;

torques = [2.5e-3, 3e-3, 3.25e-3, 3.5e-3];

if 1
	closedResponse1 = cell(size(torques));
	closedResponse2 = cell(size(torques));
	phases1 = cell(size(torques));
	phases2 = cell(size(torques));

	i = 1;
	for dTorqueAmp = torques
		dTorqueAmp
		Cgain = 10
		sim('WaterRunner_closed3');
		
		time = output.time;
		[~, angles] = output.signals.values;
		closedResponse1{i} = [time,angles];
		phase = cpg_omega.signals(2).values(:,4); 
		phases1{i} = [time,phase];
		
		dTorqueAmp
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

figure(1)
subplot(211);
	hold all
	for i = 1:length(torques)
		plot(closedResponse1{i}(:,1),closedResponse1{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Response w/ heuristic');
	ylabel('roll angle [degrees]');
	legend([num2str(torques'*1e3),repmat('mN-m',size(torques'))],'Location','EastOutside');
	axis([0, 10, -45 45]);
subplot(212);
	hold all
	for i = 1:length(torques)
		plot(phases1{i}(:,1),phases1{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Leg Phase Relationship');
	xlabel('time');
	ylabel('phase [radians]');
	legend([num2str(torques'*1e3),repmat('mN-m',size(torques'))],'Location','EastOutside');
	axis([0, 10, 0, 2*pi]);


figure(2)
subplot(211);
	hold all
	for i = 1:length(torques)
		plot(closedResponse2{i}(:,1),closedResponse2{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Response w/o heuristic');
	ylabel('roll angle [degrees]');
	legend([num2str(torques'*1e3),repmat('mN-m',size(torques'))],'Location','EastOutside');
	axis([0, 10, -45 45]);
subplot(212);
	hold all
	for i = 1:length(torques)
		plot(phases2{i}(:,1),phases2{i}(:,2),'LineWidth',2);
	end
	hold off
	title('Leg Phase Relationship');
	xlabel('time');
	ylabel('phase [radians]');
	legend([num2str(torques'*1e3),repmat('mN-m',size(torques'))],'Location','EastOutside');
	axis([0, 10, 0, 2*pi]);
