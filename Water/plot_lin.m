load('speed_df.mat')

figure(1)
for i =1:5
	scatter(DF(i,:),robot_speed(i,:))
	hold on
end
hold off
legend('$\omega = 60 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$', '$\omega = 70 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$','$\omega = 80 \frac{rad}{s}$', 'Interpreter','LaTex') 
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('Robot speed [m/s]')
grid on

figure(2)
for i = 1:5
	h1 = scatter(DF(i,:),robot_vert_avg(i,:));
	hold on
end
hold off
legend('$\omega = 60 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$', '$\omega = 70 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$','$\omega = 80 \frac{rad}{s}$', 'Interpreter','LaTex') 
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('Robot vertical position [m]')
grid on

figure(3)
for i = 1:5
	scatter(DF(i,:),COT(i,:))
	hold on
end
hold off
legend('$\omega = 60 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$', '$\omega = 70 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$','$\omega = 80 \frac{rad}{s}$', 'Interpreter','LaTex') 
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('Cost of Transport')

figure(4)
for i = 1:5
	scatter(robot_speed(i,:),COT(i,:),'o')
	hold on
end
hold off
legend('$\omega = 60 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$', '$\omega = 70 \frac{rad}{s}$', '$\omega = 65 \frac{rad}{s}$','$\omega = 80 \frac{rad}{s}$', 'Interpreter','LaTex') 
xlabel('Robot Speed [m/s]')
ylabel('Cost of Transport')

