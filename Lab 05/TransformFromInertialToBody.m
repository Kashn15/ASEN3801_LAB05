%% TransformFromInertialToBody Function
% Creator: Bridger and Natsumi 
function [wind_body] = TransformFromInertialToBody(wind_inertial, eul_angle)

phi = eul_angle(1,1); % roll 
theta = eul_angle(2,1); % pitch
psi = eul_angle(3,1); % yaw
 
R_BE = Rot_Mat(phi, theta, psi); % Rotation Matrix Body to Inertial
R_EB = R_BE.'; % Compute the transpose of the rotation matrix

wind_body = R_EB * wind_inertial; % Transform wind vector from inertial to body frame

end