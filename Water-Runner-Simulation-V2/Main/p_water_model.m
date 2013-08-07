function output = p_water_model(u)

% u = [y_c angle v_n  ang_vel y_w]
% ind  1    2      3  4        5

%Read in variables
water_level = u(5);

position = u(1);  % y posiiton, center of foot pad
velocity = u(3);   %normal velocity to center of food pad

theta = u(2);
omega = u(4);

%Hard coded parameters to be replaced with globals
radius = 0.02;
dragCoeff = 0.703;
density = 1000;
gravity = -9.81;

if theta == 0, theta = eps;end
percentSub = 0.5 + (water_level-position)/(2*radius*sin(theta));
if percentSub < 0,    percentSub = 0;
elseif percentSub >1, percentSub = 1;
elseif percentSub
end


force_k = integral(@force_sk, -1*radius, -1*radius + 2*percentSub*radius);
force_b = integral(@force_sb, -1*radius, -1*radius + 2*percentSub*radius);
output = [force_k -force_b];   % be ware of negative


    function force = force_sk(s)
        %computes d(drag)/ds at point s
        force = dragCoeff*density*sqrt(radius^2 - s.^2).*(-2*gravity*depth_s(s));
    end

    function force = force_sb(s)
        %computes d(drag)/ds at point s
        force = dragCoeff*density*sqrt(radius^2 - s.^2).*(normalVel_s(s) .* abs(normalVel_s(s)));
    end

    function depth = depth_s(s)
        depth = (water_level - position) - s * sin(theta);
    end

    function vel = normalVel_s(s)
        %returns component of velocity normal to the footpad
        vel = omega*s + velocity;
    end

end