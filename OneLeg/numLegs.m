SimParam;

forceRatio = (r2/r1)^2;

T_des = 1000;

LEGS = 1:10;
y = nan(1,length(LEGS));
power = nan(1,length(LEGS));

for k = 1 : numel(LEGS)

    y(k) = real(SSheight(freq,Amp,r1,mass,forceRatio,k)+leg_length);
	if y(k) < 0
		y(k) = nan;
	else
		power(k) = k*SSpower(y(k)-leg_length,freq,Amp,r1,forceRatio);
	end
end

save('numlegs.mat')

figure(1)
plot(LEGS,y)
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Number of Legs')
ylabel('Height [m]')
set(gca, 'Color', 'None')

figure(2)
plot(LEGS,power)
%axis([0, 100, 0, .1, -.1, .2])
xlabel('Number of Legs')
ylabel('Power [W]')
set(gca, 'Color', 'None')
