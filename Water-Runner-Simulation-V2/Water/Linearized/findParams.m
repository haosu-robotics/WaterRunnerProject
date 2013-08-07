global r1 forceRatio mass df
simParam2;

load_system('WaterRunner_open.slx');

freqs = 55:5:80;
y_avg = nan(size(freqs));
y_pred = nan(size(freqs));

for k = 1 : numel(freqs)
    freq = freqs(k)
	w = freq;
	omega_0 = freq*ones(4,1);
	phase_0 = [0 pi 0 pi];
   
	w1 = freq/(2*df) 
	w2 = freq/(2*(1-df))
	T_des = 5;
	try
		T_sim = sim('WaterRunner_open.slx',T_des);
	catch
		continue;
	end

    if(T_sim(end) < T_des)
		continue;
	end
    
	%find steady state height
	[Y_sim, ~]  = output.signals.values;
	if(Y_sim(end) < 0)
		continue;
	end
	Y_sim(T_sim <T_sim(end)/2) = [];
	y_avg(k) = mean(Y_sim)
end
save('omega.mat','y_avg','freqs')

%%%% Fitting %%%%
coeff = nlinfit(freqs,y_avg,@fitParams,[.04,.7]);

A = real(coeff(1))
area = real(coeff(2))
Amp = A;

r1 = 0.02;
r2 = r1/4;
S1 = pi*r1^2*area;

density = 1000;
g = 9.81;
C_d = 0.703;

b_water = 0.5*C_d*S1*density;
k_water = C_d*S1*density*g;
Fratio = 1/16;

k=1
for w=freqs
	y_pred(k) = findSSheight(w,w,b_water,k_water,Amp,Fratio,mass*g/4,y_avg(k)-Amp)+Amp;
	k = k+1;
end

figure(1)
plot(freqs,y_avg,'b',freqs,y_pred,'g')
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Frequency [rad/s]')
ylabel('Height [m]')
lgd = legend('Robot Simulation Height','Calculated Height','Location','NorthEast');
set(lgd, 'Color', 'None')
saveas(gcf,'ParamFit.fig')
