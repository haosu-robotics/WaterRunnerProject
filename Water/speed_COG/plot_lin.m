load('speed_cg.mat')

figure(1)
scatter3(SS(:),CG(:),robot_speed(:))
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('Robot speed [m/s]')
grid on

figure(2)
h1 = scatter3(SS(:),CG(:),robot_vert_low(:));
hold on
h2 = scatter3(SS(:),CG(:),robot_vert_high(:));
hold off
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('Robot vertical position [m]')
grid on

%{
figure(3)
surf(SS,CG,robot_pow)
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('average power [W]')
grid on
%}

figure(4)
scatter3(SS(:),CG(:),COT(:))
xlabel('Rotation speed [rad/s]')
ylabel('COG [m]')
zlabel('Cost of Transport')

figure(5)
scatter(robot_speed(:),COT(:),'o')
xlabel('Robot Speed [m/s]')
ylabel('Cost of Transport')

