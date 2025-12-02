% Rotational Kinematics
% Creators: Bridger, Natsumi, Drake, and Sayer

function eul_dot = euler_kinematics(phi, theta, psi, pqr)

p = pqr(1); % Roll
q = pqr(2); % Pitch
r = pqr(3); % Yaw

R1 = [1 , sin(phi)*tan(theta), cos(phi)*tan(theta)];
R2 = [0, cos(phi), -sin(phi)];
%R3 = [0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
R3 = [0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

T = [R1; R2; R3];

eul_dot = T * [p; q; r];

end     