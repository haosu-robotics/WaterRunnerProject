%1-D sweep of frequency

SimParam;

load_system('water_hopper_df.mdl');
T_des = 20;

freqs = 12:2:100;
df = 0.5;

y_sim = nan(size(freqs));
y_pred = nan(size(freqs));
power_sim = nan(size(freqs));
power_pred = nan(size(freqs));
y_pred2 = nan(size(freqs));
power_pred2 = nan(size(freqs));

%robot_vert_avg =  open('height1.mat');
%rheights3 = robot_vert_avg.robot_vert_avg;

for k = 1 : numel(freqs)
    freq = freqs(k)

    y_pred(k) = real(SSheight(freq,df,Amp,r1,mass,forceRatio,area)+leg_length);
	if y_pred(k) < 0
		y_pred(k) = nan;
	else
		power_pred(k) = SSpower(y_pred(k)-leg_length,freq,Amp,r1,forceRatio);
	end

	y_pred2(k) = real(SSheight2(freq,df,Amp,r1,mass,forceRatio,area)+leg_length);
	if y_pred2(k) < 0
		y_pred2(k) = nan;
	end
	%{
		power_pred2(k) = SSpower(y_pred2(k)-leg_length,freq,Amp,r1,forceRatio);
	end
	%}

    y_0 = Amp/2;
	T_sim = sim('water_hopper_df.mdl',T_des);

    if(T_sim(end) < T_des)
		continue;
	end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball = ball_position(:,2);
	%Work = Work(:,2);
    %%%% Trimming %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball(T_sim <T_sim(end)/2) = [];
	%Work(T_sim <T_sim(end)/2) = [];
	T_sim(T_sim <T_sim(end)/2) = [];
    T_sim = T_sim - T_sim(1);

	y_sim(k) = mean(Y_ball);
	if y_sim(k) < 0
		y_sim(k) = nan;
	end
	%{
	else
		fit2 = polyfit( T_sim, Work,1);
		power_sim(k) = fit2(1);
	end
	%}

end

save('oneleg1.mat')

figure(3)
plot(12:2:100,rheights3,'k')
hold on
%plot(freqs,y_sim,'b')
plot(freqs,y_pred,'r')
%plot(freqs,y_pred2,'r')
hold off
axis([40, 100, 0 0.035])
xlabel('Frequency [rad/s]')
ylabel('Height [m]')
lgd = legend('Simulated Robot Height','Best-Fit Height','Location','NorthWest','boxoff');
%title('Amplitude = 0.0249 m, Projected Area = 0.9185')
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
saveas(gcf,'heightdf.fig')

%{
figure(4)
plot(freqs,abs(y_sim-rheights3)./rheights3*100,'b')
hold on
plot(freqs,abs(y_pred-rheights3)./rheights3*100,'g')
plot(freqs,abs(y_pred2-rheights3)./rheights3*100,'r')
axis([40, 100, 0 100])
hold off
xlabel('Frequency [rad/s]')
ylabel('Percent Error')
lgd = legend('Simple Simulation Height','Numerially Calculated Height','Analytically Calculated Height','Location','NorthEast');
title('Amplitude = 0.0260 m, Projected Area = 0.7968')
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
saveas(gcf,'heightdferr.fig')
%}
%{
figure(3)
plot(freqs,power_sim,'b')
hold on
plot(freqs,power_pred,'g')
plot(freqs,power_pred2,'r')
hold off
xlabel('Frequency [rad/s]')
ylabel('Power [W]')
lgd = legend('Simulated Power','Predicted Power 1','Predicted Power 2','Location','NorthWest');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')

figure(4)
plot(freqs,abs(power_pred-power_sim)./power_sim*100,'g')
hold on
plot(freqs,abs(power_pred2-power_sim)./power_sim*100,'r')
hold off
xlabel('Frequency [rad/s]')
ylabel('Percent Error')
lgd = legend('Predicted Power Error 1','Predicted Power Error 2','Location','NorthWest');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
%}


