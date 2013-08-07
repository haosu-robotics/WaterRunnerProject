load('speed_df.mat')

figure(1)
surf(SS,DF,robot_speed,'FaceColor','interp')
hold on
plot3(SS(DF == 0.5), DF(DF == 0.5), robot_speed(DF == 0.5),'k','LineWidth', 3)
hold off
xlabel('Rotation speed [rad/s]')
ylabel('Duty Factor')
zlabel('Robot speed [m/s]')
set(gca, 'Color', 'None')
xlim([20,100]); ylim([0 1])
grid on

figure(2)
surf(SS,DF,robot_vert_low,'FaceColor','g','FaceAlpha',0.75);
hold on
surf(SS,DF,robot_vert_high,'FaceColor','b','FaceAlpha',0.5);
plot3(SS(DF == 0.5), DF(DF == 0.5), robot_vert_low(DF == 0.5),'g','LineWidth', 3)
plot3(SS(DF == 0.5), DF(DF == 0.5), robot_vert_high(DF == 0.5),'b','LineWidth', 3)
hold off
xlabel('Rotation speed [rad/s]')
ylabel('Duty Factor')
zlabel('Robot min vertical position [m]')
grid on
xlim([20,100]);
ylim([0 1]);
set(gca, 'Color', 'None')

figure(3)
surf(SS,DF,COT,'FaceColor','interp')
hold on
plot3(SS(DF == 0.5), DF(DF == 0.5), COT(DF == 0.5),'k','LineWidth', 3)
xlabel('Rotation speed [rad/s]')
ylabel('Duty Factor')
zlabel('Cost of Transport')
xlim([20,100]); ylim([0 1])
set(gca, 'Color', 'None')

figure(4)
scatter(robot_speed(:),COT(:),'o')
xlabel('Robot Speed [m/s]')
ylabel('Cost of Transport')
%axis([0 3 0 3])
set(gca, 'Color', 'None')
