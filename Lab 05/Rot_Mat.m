%% Rotation Matrix Function
% Creator: Natsumi Kakuda

function R_BE = Rot_Mat(phi, theta, psi)

R1 = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)];
R2 = [cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)];
R3 = [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];

R_BE = [R1; R2; R3]; % Body to Earth

end