%% WindAnglesFromFromVelocityBody Function

function [wind_angles] = WindAnglesFromVelocityBody(velocity_body)

u = velocity_body(1);
v = velocity_body(2);
w = velocity_body(3);

V = norm(velocity_body);

if V < 1e-6
    alpha = 0;
    beta = 0;
else
    alpha = atan2(w,u);
    beta = asin(v / V);
end

wind_angles = [V; beta; alpha]; % alpha and beta are in [rads]

end