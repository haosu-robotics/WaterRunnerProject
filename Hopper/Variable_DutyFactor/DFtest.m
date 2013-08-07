clc
close all
clear all

DF = linspace(0,1);
forceRatio = 1/16;

for DF0 = 0.2:0.02:0.9
	F =  (1./DF - forceRatio./(1-DF));
	F_ap =(1/DF0-forceRatio/(1-DF0)) + (-DF0^-2 - forceRatio*(1-DF0)^-2)*(DF);
	F_ap2 =(1/DF0-forceRatio/(1-DF0)) + (-DF0^-2 - forceRatio*(1-DF0)^-2)*(DF - DF0);
	cla
    plot(DF,F,DF,F_ap,DF,F_ap2)
	axis([0 1 -20 20])
    grid on
	legend('F','Fap','F_ap2')
    drawnow
	pause(.01)
end
