figure(1)
t = ball_position(:,1);
y1 = ball_position(:,2);
y2 = ball_position(:,3);

plot(t,y1)
hold on
plot(t,y2,'r','LineWidth',3)
plot(t,0.005*heaviside(t - 3)+.01,'k--')
hold off
title('Step Response')
ylabel('Height [m]')
xlabel('Time [t]')
legend('Robot Height','Filtered Height','Desired Height','Location','SouthEast')

figure(2)

t = omega_control.time;
w1 = omega_control.signals.values(1,1,:);
w1 = reshape(w1,length(w1),1);
w2 = omega_control.signals.values(2,1,:);
w2 = reshape(w2,length(w2),1);
plot(t,w1,'b','LineWidth',4)
hold on
plot(t,w2,'r','LineWidth',1)
hold off
title('Step Response')
ylabel('Frequency [rad/s]')
xlabel('Time [t]')
legend('Omega Down','Omega Up','Location','SouthEast')

