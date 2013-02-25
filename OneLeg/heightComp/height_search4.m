SimParam;

forceRatio = (r2/r1)^2;

load_system('water_hopper.mdl');
T_des = 20;

freqs = 12:8:100;
amps = 0.02:0.0025:0.035;
areas = 0.5:0.025:1;

[FREQ, AMPS, AREA] = meshgrid(freqs,amps,areas);

y_sim = zeros(size(FREQ));

open ../../Water-Runner-Simulation/Water/height.mat

robot_vert_avg =  open('../../Water-Runner-Simulation/Water/height.mat');
rheights3 = robot_vert_avg.robot_vert_avg;
rheights3(isnan(rheights3)) = 0;

for k = 1 : numel(FREQ)
    freq = FREQ(k)
	Amp = AMPS(k)
	area = AREA(k)

	S1 = (pi*r1^2)*area;
	S2 = (pi*r2^2)*area;
    
	leg_length = Amp;
	y_0 = Amp/2;
	T_sim = sim('water_hopper.mdl',T_des);

    if(T_sim(end) < T_des)
		continue;
	end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball = ball_position(:,2);
	Work = Work(:,2);
    %%%% Trimming %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball(T_sim <T_sim(end)/2) = [];
	Work(T_sim <T_sim(end)/2) = [];
	T_sim(T_sim <T_sim(end)/2) = [];
    T_sim = T_sim - T_sim(1);

	y_sim(k) = mean(Y_ball);
	if y_sim(k) < 0
		y_sim(k) = 0;
	else
		fit2 = polyfit( T_sim, Work,1);
		power_sim(k) = fit2(1);
	end
	
end

besterr = realmax;
for i = 1:length(amps)
	for j = 1:length(areas)
		err = norm(y_sim(i,:,j) - rheights3);
		if err < besterr
			besterr = err
			bestamp = amps(i);
			bestarea = areas(j);
		end
	end
end

bestamp
bestarea
save('oneleg.mat')

%{
figure(2)
surf(FREQ,AREA,y_sim,'FaceAlpha',0.5,'FaceColor','b')
hold on
surf(FREQ,AREA,rheights2,'FaceColor','g')
hold off
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Amplitude [m]')
zlabel('Height [m]')
lgd = legend('Hopper Height','Robot Model Height');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
%}
