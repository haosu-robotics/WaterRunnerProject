SimParam;

forceRatio = (r2/r1)^2;

load_system('../water_hopper.mdl');
T_des = 20;

freqs = 12:4:100;
amps = 0.01:0.01:0.1;

[FREQ, AMP] = meshgrid(freqs,amps);


y_sim = nan(size(FREQ));
y_pred = nan(size(FREQ));
power_sim = nan(size(FREQ));
power_pred = nan(size(FREQ));

for k = 1 : numel(FREQ)
    freq = FREQ(k)
	Amp = AMP(k)
	leg_length = Amp;

    y_pred(k) = real(SSheight(freq,Amp,r1,mass,forceRatio,1)+leg_length);
	if y_pred(k) < 0
		y_pred(k) = nan;
	else
		power_pred(k) = SSpower(y_pred(k)-leg_length,freq,Amp,r1,forceRatio);
	end

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
		y_sim(k) = nan;
	else
		fit2 = polyfit( T_sim, Work,1);
		power_sim(k) = fit2(1);
	end

end

save('oneleg.mat')

figure(1)
surf(FREQ,AMP,y_sim,'FaceAlpha',0.5,'FaceColor','b')
hold on
surf(FREQ,AMP,y_pred,'FaceAlpha',.5,'FaceColor','g')
hold off
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Amplitude [m]')
zlabel('Height [m]')
lgd = legend('Simulated Height','Predicted Height');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')

figure(2)
surf(FREQ,AMP,power_sim,'FaceAlpha',0.5,'FaceColor','b')
hold on
surf(FREQ,AMP,power_pred,'FaceAlpha',.5,'FaceColor','g')
hold off
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Amplitude [m]')
zlabel('Power [W]')
lgd = legend('Simulated Power','Predicted Power');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
