%% PlotAircraftSIM
% For Lab 05 developed off of Lab 04
% Creator: Natsumi Kakuda

%% Notes
% This code uses NED positions for x,y,z in the inertial frame where z_e <
% 0. With the Aircraft body frame using the standard convention of x_hat is
% out of the nose of the Aircraft, y_hat is out of the pilot's right wing,
% and z_hat is direction positive downwards towards the Earth's core.

%% Plot function
function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

Inertial_pos = aircraft_state_array(:,1:3);
Euler_angles = aircraft_state_array(:,4:6);
BF_velo = aircraft_state_array(:,7:9);
Angular_velo = aircraft_state_array(:,10:12);

%% Plot inertial positions
figure(fig(1));
subplot(3,1,1);
plot(time,Inertial_pos(:,1),col); 
hold on;
title('X Position');
xlabel('Time [s]')
ylabel('x [m]')
ytickformat('%.3f')
grid on;

subplot(3,1,2)
plot(time,Inertial_pos(:,2),col); 
hold on;
title('Y Position');
xlabel('Time [s]')
ylabel('y [m]')
ytickformat('%.3f')
grid on;

subplot(3,1,3)
plot(time,-Inertial_pos(:,3),col);
hold on;
title('Z Position');
xlabel('Time [s]')
ylabel('z [m]')
ytickformat('%.3f')
grid on

sgtitle('Inertial Postions', 'FontSize', 13);

%% Plot Euler Angles
figure(fig(2));

subplot(3,1,1)
plot(time,Euler_angles(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('\phi [rad]')
ytickformat('%.3f')
grid on;
title('Roll');

subplot(3,1,2)
plot(time,Euler_angles(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('\theta [rad]')
ytickformat('%.3f')
grid on
title('Pitch');

subplot(3,1,3)
plot(time,Euler_angles(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('\psi [rad]')
ytickformat('%.3f')
grid on;
title('Yaw');

sgtitle('Euler Angles', 'FontSize', 13);

%% Plot Body-Frame Velocity
figure(fig(3));

subplot(3,1,1)
plot(time,BF_velo(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('u [m/s]')
ytickformat('%.3f')
grid on
title('X-directional Velocity');

subplot(3,1,2)
plot(time,BF_velo(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('v [m/s]')
ytickformat('%.3f')
grid on
title('Y-directional Velocity');

subplot(3,1,3)
plot(time,BF_velo(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('w [m/s]')
ytickformat('%.3f')
grid on
title('Z-directional Velocity');

sgtitle('Body-Frame Velocities', 'FontSize', 13);

%% Plot Angular Velocities
figure(fig(4));

subplot(3,1,1)
plot(time,Angular_velo(:,1),col); 
hold on;
xlabel('Time [s]')
ylabel('p [rad/s]')
ytickformat('%.3f')
title('Roll Rate');
grid on;

subplot(3,1,2)
plot(time,Angular_velo(:,2),col); 
hold on;
xlabel('Time [s]')
ylabel('q [rad/s]')
ytickformat('%.3f')
grid on
title('Pitch Rate');

subplot(3,1,3)
plot(time,Angular_velo(:,3),col); 
hold on;
xlabel('Time [s]')
ylabel('r [rad/s]')
ytickformat('%.3f')
grid on
title('Yaw Rate');

sgtitle('Angular Velocities', 'FontSize', 13);

%% Plot control variables (Control surfaces deflection)
if ~isempty(control_input_array)
figure(fig(5));

% Conversion of Rad into Deg
elevator_deg = rad2deg(control_input_array(:,1));   % elevator
aileron_deg  = rad2deg(control_input_array(:,2));   % aileron
rudder_deg   = rad2deg(control_input_array(:,3));   % rudder
throttle     = control_input_array(:,4);            % stays as fraction 0–1

subplot(4,1,1);
plot(time,elevator_deg,col); 
hold on;
xlabel('Time [s]');
ylabel('Elevator \delta_e [deg]');
title('Elevator');
grid on;

subplot(4,1,2);
plot(time, aileron_deg,col); 
hold on;
xlabel('Time [s]');
ylabel('Aileron \delta_a [deg]');
title('Aileron');
grid on;

subplot(4,1,3);
plot(time,rudder_deg,col); 
hold on;
xlabel('Time [s]');
ylabel('Rudder \delta_r [deg]');
title('Rudder');
grid on;

subplot(4,1,4);
plot(time,throttle,col); 
hold on;
xlabel('Time [s]');
ylabel('Throttle \delta_t');
ylim([0,1]);
title('Throttle');
grid on;

sgtitle('Deflection of Control Surfaces', 'FontSize', 13);

else

end

%% 3D Plot of Aircraft Position
figure(fig(6));
plot3(Inertial_pos(:,1), Inertial_pos(:,2), -Inertial_pos(:,3), col); 
grid on;
ytickformat('%.3f')
xtickformat('%.3f')
ztickformat('%.3f')
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
%Begin and end circles
hold on;
plot3(Inertial_pos(1,1), Inertial_pos(1,2), -Inertial_pos(1,3),'go', 'MarkerFaceColor','g') % Start
plot3(Inertial_pos(end,1), Inertial_pos(end,2), -Inertial_pos(end, 3),'ro', 'MarkerFaceColor','r') % End
title('3D Plot of Position', 'FontSize', 13);

end