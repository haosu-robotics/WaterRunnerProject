function y = Foot(u)

    % u = [x y z   x' y' z' Rx'  Ry'  Rz' rot1 ... rot9    angle0 water_level]
    % ind  1 2 3   4  5  6   7   8     9   10       18     19      20

    %Read in variables
	water_level = u(end);

    position = [-1*u(1), u(2)];  %x, y posiiton
    velocity = [-1*u(4), u(5)];  %x, y veolcity
	
    theta = -1*(atan2(u(11),u(10)) - pi/2);
    omega = -1*u(9);
 
    %Hard coded parameters to be replaced with globals
	radius = 0.03;
    dragCoeff = 1.1;
    density = 1000.;
    gravity = -9.81;
       
	%initialize forces
    force_x = 0;
    force_y = 0;
    force_z = 0;
    torque_x = 0;
    torque_y = 0;
    torque_z = 0;
	
	%calculate velocity component normal to footpad to determine footpad folded/unfolded state
	if y_bf() > 0
		force_x = 0;
		force_y = 0;
		force_z = 0;
		torque_x = 0;
		torque_y = 0;
		torque_z = 0;
		
		F_6DOF = [torque_x torque_y torque_z force_x force_y force_z];
		y = F_6DOF;
		return
	%if unfolded and below water
	else
		if theta >=0
			percentSub = -1 * y_bf() / (-2 *radius * sin(theta + pi));
		else
			percentSub = -1 * y_bf() / (-2 *radius * sin(theta));
		end
		if percentSub > 1
			percentSub = 1;
		end
		normal = [sin(theta), -1*cos(theta)];
		force = integral(@force_s, -1*radius, -1*radius + 2*percentSub*radius);
		moment = integral(@moment_s, -1*radius, -1*radius + 2*percentSub*radius);
	end

    force_x = force*sin(theta);
	force_y = force*cos(theta);
    torque_z = moment;
    
    F_6DOF = [torque_x torque_y torque_z force_x force_y force_z ];
    y = F_6DOF;
    
    
    function force = force_s(s)
		%computes d(drag)/ds at point s
        force = dragCoeff*density*sqrt(radius^2 - s.^2).*(normalVel_s(s) .* abs(normalVel_s(s)));
    end

    function moment = moment_s(s)
        %computes d(drag)/ds at point s
        moment = dragCoeff*density*sqrt(radius^2 - s.^2).*(normalVel_s(s) .* abs(normalVel_s(s))).*(-1*s);
    end

    function depth = depth_s(s)
        %returns depth of point s
        depth = -1*y_bf() * (1 - (s + radius)/(2*percentSub*radius));
    end

    function location = y_bf()
        %returns location of bottom of foot with respect to water level
        if theta >= 0
			location = position(2) + radius*sin(theta + pi) - water_level;
		else
			location = position(2) + radius*sin(theta) - water_level;
		end
    end

    function vel = normalVel_s(s)
        %returns component of velocity normal to the footpad
        vel = normal*velocity_s(s);
    end

    function vel = velocity_s(s)
        % returns velocity of point s by fining velocities of top and bottom 
        % of foot and linearly interpolating

        v_bf = velocity + radius*[-1*sin(theta + pi), cos(theta + pi)]*omega;
        v_tf = velocity - radius*[-1*sin(theta + pi), cos(theta + pi)]*omega;

        velDiff = (v_tf - v_bf)/(2*radius);
        velMean = (v_tf + v_bf)/2;

        velx = velDiff(1)*s + velMean(1);
        vely = velDiff(2)*s + velMean(2);
        vel = [velx; vely];
    end
end

