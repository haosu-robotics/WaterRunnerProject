%2D sweep
SimParam;

forceRatio = (r2/r1)^2;

load_system('water_hopper_df.mdl');

freqs = 12:8:100;
dfs = 0.05:.15:.95;

[FREQ, DF] = meshgrid(freqs,dfs);


y_sim = nan(size(FREQ));
y_pred = nan(size(FREQ));
power_sim = nan(size(FREQ));
power_pred = nan(size(FREQ));
y_pred2 = nan(size(FREQ));
power_pred2 = nan(size(FREQ));

for k = 1 : numel(FREQ)
    freq = FREQ(k)
	df = DF(k)
	leg_length = Amp;
    
	y_pred(k) = real(SSheight(freq,df,Amp,r1,mass,forceRatio,area)+leg_length);
	if y_pred(k) < 0
		y_pred(k) = nan;
	end

	y_pred2(k) = real(SSheight2(freq,df,Amp,r1,mass,forceRatio,area)+leg_length);
	if y_pred2(k) < 0
		y_pred2(k) = nan;
	end
	
    y_0 = Amp/2;
	try
		T_sim = sim('water_hopper_df.mdl',T_des);
	catch
		continue;
	end

    if(T_sim(end) < T_des)
		continue;
	end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball = ball_position(:,2);
    %%%% Trimming %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Y_ball(T_sim <T_sim(end)/2) = [];
	y_sim(k) = mean(Y_ball);
	if y_sim(k) < 0 || y_sim(k) > .100
		y_sim(k) = nan;
	end
	%}
end

save('oneleg.mat')

figure(1)
surf(FREQ,DF,y_sim,'FaceAlpha',0.5,'FaceColor','b')
hold on
surf(FREQ,DF,y_pred,'FaceAlpha',.5,'FaceColor','g')
surf(FREQ,DF,y_pred2,'FaceAlpha',.5,'FaceColor','r')
hold off
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Duty Factor [m]')
zlabel('Height [m]')
lgd = legend('Simulated Height','Predicted Height 1','Predicted Height 2','Location','NorthEast');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
saveas(gcf,'heightdf.fig')

figure(2)
ax(1) = subplot(121);
	h = pcolor(FREQ,DF,abs(y_pred-y_sim)./y_sim*100);
	set(h,'FaceColor','interp')
	title('Numerically Calculated Height Error')
	caxis([0 50])
	axis([44 100 .2 .8])
	xlabel('Frequency [rad/s]')
	ylabel('Duty Factor')
	set(gca, 'Color', 'None')
ax(2) = subplot(122);
	h = pcolor(FREQ,DF,abs(y_pred2-y_sim)./y_sim*100);
	set(h,'FaceColor','interp')
	title('Analytically Calculated Height Error')
	axis([44 100 .2 .8])
	caxis([0 50])
	xlabel('Frequency [rad/s]')
	ylabel('Duty Factor')
h=colorbar;
set(h, 'Position', [.8314 .11 .0581 .8150])
for i=1:2
	pos=get(ax(i), 'Position');
	set(ax(i), 'Position', [pos(1) pos(2) 0.75*pos(3) pos(4)]);
end
set(gca, 'Color', 'None')
%clabel('Percent Error')
saveas(gcf,'heightdferr.fig')


%{
figure(3)
surf(FREQ,DF,power_sim,'FaceAlpha',0.5,'FaceColor','b')
hold on
surf(FREQ,DF,power_pred,'FaceAlpha',.5,'FaceColor','g')
surf(FREQ,DF,power_pred2,'FaceAlpha',.5,'FaceColor','r')
hold off
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Amplitude [m]')
zlabel('Power [W]')
lgd = legend('Simulated Power','Predicted Power 1','Predicted Power 2','Location','NorthEast');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')

figure(4)
surf(FREQ,DF,abs(power_pred-power_sim)./power_sim*100,'FaceAlpha',.5,'FaceColor','g')
hold on
surf(FREQ,DF,abs(power_pred2-power_sim)./power_sim*100,'FaceAlpha',.5,'FaceColor','r')
hold off
axis([0, 100, 0, .1, 0, 25])
xlabel('Frequency [rad/s]')
ylabel('Amplitude [m]')
zlabel('Percent Error')
lgd = legend('Predicted Power Error 1','Predicted Power Error 2','Location','NorthEast');
set(gca, 'Color', 'None')
set(lgd, 'Color', 'None')
%}

