% Rotational Kinematics
function eul_dot = euler_kinematics(phi, theta, psi, pqr)

p = pqr(1); % Roll
q = pqr(2); % Pitch
r = pqr(3); % Yaw
 
cphi = cos(phi); 
sphi = sin(phi);
cth  = cos(theta); 
sth = sin(theta);

T = [ 1, sphi*sth/cth, cphi*sth/cth;   % = [1, sin(phi)*tan(theta), cos(phi)*tan(theta)]
      0, cphi, -sphi;
      0, sphi/cth, cphi/cth];

eul_dot = T * [p; q; r];

end     