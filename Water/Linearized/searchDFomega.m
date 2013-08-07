simParam2;

load_system('WaterRunner_open4.slx');

freqs = 55:5:75;
dfs = 0.4:.05:.6;
[FREQ, DF] = meshgrid(freqs,dfs);

y_avg = nan(size(FREQ));
y_pred = nan(size(FREQ));

for k = 1 : numel(FREQ)
    freq = FREQ(k)
	w = freq;
	omega_0 = freq*ones(4,1);
	phase_0 = [0 pi 0 pi];
	df = DF(k)
   
	w1 = freq/(2*df) 
	w2 = freq/(2*(1-df))
	T_des = 5;
	try
		T_sim = sim('WaterRunner_open4.slx',T_des);
	catch
		continue;
	end

    if(T_sim(end) < T_des)
		continue;
	end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [Y_sim, ~]  = output.signals.values;
    if(Y_sim(end) < 0)
		continue;
	end
    %%%% Trimming %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_sim(T_sim <T_sim(end)/2) = [];
	y_avg(k) = mean(Y_sim)
	if y_avg(k) < 0 || y_avg(k) > .100
		y_avg(k) = nan;
	end
	y_pred(k) = real(findSSheight(w1,w2,b_water,k_water,Amp,Fratio,mass*g/4,y_avg(k)-Amp))+Amp
	if y_pred(k) < 0
		y_pred(k) = nan;
	end

end

save('omega_df.mat')

figure(1)
surf(FREQ,DF,y_avg,'FaceAlpha',0.5,'FaceColor','b')
hold on
surf(FREQ,DF,y_pred,'FaceAlpha',.5,'FaceColor','g')
hold off
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Duty Factor [m]')
zlabel('Height [m]')
lgd = legend('Robot Simulation Height','Calculated Height','Location','NorthEast');
set(lgd, 'Color', 'None')
saveas(gcf,'heightdf.fig')

figure(2)
	h = pcolor(FREQ,DF,abs(y_avg - y_pred)./y_avg*100);
	set(h,'FaceColor','interp')
	title('Percent Error')
	caxis([0 50])
	axis([44 100 .2 .8])
	xlabel('Frequency [rad/s]')
	ylabel('Duty Factor')
	set(gca, 'Color', 'None')
h=colorbar;
set(h, 'Position', [.8314 .11 .0581 .8150])
%clabel('Percent Error')
