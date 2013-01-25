load('speed.mat')

figure(1)
plot(SS,robot_speed)
hold on
hold off
xlabel('Rotation speed [rad/s]')
ylabel('Robot speed [m/s]')
set(gca, 'Color', 'None')
grid on

figure(2)
plot(SS,robot_vert_low,'g');
hold on
plot(SS,robot_vert_high,'b');
hold off
xlabel('Rotation speed [rad/s]')
ylabel('Robot min vertical position [m]')
grid on
set(gca, 'Color', 'None')

figure(3)
plot(SS,COT)
hold on
xlabel('Rotation speed [rad/s]')
ylabel('Cost of Transport')
set(gca, 'Color', 'None')

figure(4)
scatter(robot_speed,COT,'o')
xlabel('Robot Speed [m/s]')
ylabel('Cost of Transport')
set(gca, 'Color', 'None')
