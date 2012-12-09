 function y = calcAngle(u)
 
 % u = [x y z   x' y' z' Rx'  Ry'  Rz' rot1 ... rot9    angle0 water_level]
    % ind  1 2 3   4  5  6   7   8     9   10       18     19      20

    %Read in variables
    l3_angle_0 = u(end-1);
    theta = atan2(u(11),u(10))+l3_angle_0;
	y = theta	
