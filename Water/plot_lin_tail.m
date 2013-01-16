load('speed_tail.mat')

figure(1)
surf(SS,180/pi*TA,robot_speed,'FaceColor','interp')
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
zlabel('Robot speed [m/s]')
set(gca, 'Color', 'None')
xlim([40,80]); ylim([,0,80])
grid on

figure(2)
surf(SS,180/pi*TA,robot_vert_low,'FaceColor','g','FaceAlpha',0.75);
hold on
surf(SS,180/pi*TA,robot_vert_high,'FaceColor','b','FaceAlpha',0.5);
hold off
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
zlabel('Robot vertical position [m]')
grid on
xlim([40,80]); ylim([,0,80])
set(gca, 'Color', 'None')

figure(3)
surf(SS,180/pi*TA,robot_angle,'FaceColor','interp','FaceAlpha',0.99)
hold on
C = contour(SS,180/pi*TA,robot_angle, [0, 0],'k','LineWidth', 3);
hold off
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
zlabel('robot angle [radians]')
grid on
set(gca, 'Color', 'None')
xlim([40,80]); ylim([,0,80])

figure(4)
COT2 = COT;
COT2(COT < 0 | COT > 5) = nan;
surf(SS,180/pi*TA,COT2,'FaceColor','interp')
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
zlabel('Cost of Transport')
xlim([40,80]); ylim([,0,80])
set(gca, 'Color', 'None')

figure(5)
scatter(robot_speed(:),COT(:),'o')
xlabel('Robot Speed [m/s]')
ylabel('Cost of Transport')
axis([0 3 0 3])
set(gca, 'Color', 'None')
