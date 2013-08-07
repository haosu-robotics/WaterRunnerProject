load('speed_tail.mat')

figure(1)
h= pcolor(SS,180/pi*TA,robot_speed);
set(h,'FaceColor','interp')
h = colorbar;
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
ylabel(h,'Robot speed [m/s]')
hold on
C = contour(SS,180/pi*TA,robot_angle, [0, 0],'k','LineWidth', 3);
[Fx Fy] = gradient(robot_speed);
contour(SS,180/pi*TA,Fy,[0 0],'w','LineWidth',3');
hold off

set(gca, 'Color', 'None')
xlim([40,100]); ylim([0,80]); caxis([0, 3.5])
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
xlim([40,100]); ylim([0,80])
set(gca, 'Color', 'None')

figure(3)
h =  pcolor(SS,180/pi*TA,180/pi*robot_angle);
set(h,'FaceColor','interp')
h = colorbar;
hold on
C = contour(SS,180/pi*TA,180/pi*robot_angle, [0, 0],'k','LineWidth', 3);
hold off
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
ylabel(h,'robot angle [radians]')
grid on
set(gca, 'Color', 'None')
xlim([40,100]); ylim([0,80])

figure(4)
COT2 = COT;
COT2(COT < 0 | COT > 5) = nan;
h = pcolor(SS,180/pi*TA,COT2);
set(h,'FaceColor','interp')
h = colorbar;
hold on
C = contour(SS,180/pi*TA,robot_angle, [0, 0],'k','LineWidth', 3);
[Fx Fy] = gradient(robot_speed);
contour(SS,180/pi*TA,Fy,[0 0],'w','LineWidth',3');
hold off
xlabel('Rotation speed [rad/s]')
ylabel('tail angle [degrees]')
ylabel(h,'Cost of Transport')
xl
m([40,100]); ylim([0,80])
set(gca, 'Color', 'None')

figure(5)
scatter(robot_speed(:),COT(:),'o')
xlabel('Robot Speed [m/s]')
ylabel('Cost of Transport')
%axis([0 3 0 3])
set(gca, 'Color', 'None')
