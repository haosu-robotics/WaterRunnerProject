function y = Foot(u)

    % u = [z x'z' wy  theta  water_level]
    % ind  1 2 3  4   5      6   

    %Read in variables
    position = u(1) %z posiiton
    velocity = [-1*u(2), u(3)]   %x, z veolcity
	theta = u(5)
    omega = -1*u(4)
	water_level = u(6)
 
    %Hard coded parameters to be replaced with globals
	radiusDwn = 0.02;
    radiusUp = 0.005;
    dragCoeff = 0.703;
    density = 1000.;
    gravity = -9.81;
       
	%initialize forces
    force_x = 0;
    force_z = 0;
    torque_y = 0;
	
	%calculate velocity component normal to footpad to determine footpad folded/unfolded state
    normalVect = [sin(theta), -1*cos(theta)];
    normalVelComp = dot(normalVect, velocity);
    %fprintf('[%f, %f] %f\n', normalVect(1), normalVect(2), normalVelComp) 
    if normalVelComp >= 0
		radius = radiusDwn;
		%if above water forces are 0
		if y_bf() > 0
			force_x = 0;
			force_z = 0;
			torque_y = 0;
			
			F_3DOF = [force_x force_z torque_y];
			y = F_3DOF;
			return
		%if unfolded and below water
        else
			if theta >=0
				percentSub = -1 * y_bf() / (-2 *radius * sin(theta + pi));
			else
				percentSub = -1 * y_bf() / (-2 *radius * sin(theta));
			end
			if percentSub> 1
				percentSub = 1;
			end
			normal = [sin(theta), -1*cos(theta)];
			force1 = integral(@force_s, -1*radius, -1*radius + 2*percentSub*radius);
			moment1 = integral(@moment_s, -1*radius, -1*radius + 2*percentSub*radius);
            force2 = 0;
            moment2 = 0;
		end
    else
		%if folded => effective radius reduction
		radius = radiusUp;
		if y_bf() > 0
			force_x = 0;
			force_z = 0;
			torque_y = 0;
			
			F_3DOF = [force_x force_z torque_y];
			y = F_3DOF;
			return
        else
			%if folded and below water
			if theta >=0
				percentSub = -1 * y_bf() / (-2 *radius * sin(theta + pi));
			else
				percentSub = -1 * y_bf() / (-2 *radius * sin(theta));
			end
			if percentSub > 1
				percentSub = 1;
			end
			%forces applied to unfolded footpad area
			normal = [sin(theta), -1*cos(theta)];
			force1 = integral(@force_s, -1*radius, -1*radius + 2*percentSub*radius);
   			moment1 = integral(@moment_s, -1*radius, -1*radius + 2*percentSub*radius);

			%forces applied to folded footpad area
			normal = [-1*cos(theta), -1*sin(theta)];
            radius = radiusDwn;
			force2 = 0;
            moment2 = 0;
            force2 = integral(@force_s, -1*radius, -1*radius + 2*percentSub*radius);
            moment2 = integral(@moment_s, -1*radius, -1*radius + 2*percentSub*radius);
		end
    end

    force_x = -1 * force1*sin(theta) + force2*cos(theta);
    force_z =      force1*cos(theta) + force2*sin(theta);
    torque_y = moment1 + moment2;
    
	F_3DOF = [force_x force_z torque_y];
    y = F_3DOF;
    
    
    function force = force_s(s)
		%computes d(drag)/ds at point s
        force = dragCoeff*density*sqrt(radius^2 - s.^2).*(-2*gravity*depth_s(s) ...
            + normalVel_s(s) .* abs(normalVel_s(s)));
    end

    function moment = moment_s(s)
        %computes d(drag)/ds at point s
        moment = dragCoeff*density*sqrt(radius^2 - s.^2).*(-2*gravity*depth_s(s) ...
            + normalVel_s(s) .* abs(normalVel_s(s))).*(-1*s);
    end

    function depth = depth_s(s)
        %returns depth of point s
        depth = -1*y_bf() * (1 - (s + radius)/(2*percentSub*radius));
    end

    function location = y_bf()
        %returns location of bottom of foot with respect to water level
        if theta >= 0
			location = position + radius*sin(theta + pi) - water_level;
		else
			location = position + radius*sin(theta) - water_level;
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

