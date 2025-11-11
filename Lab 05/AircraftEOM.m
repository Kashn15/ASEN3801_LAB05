%% AircraftEOM Function
% Creator: Natsumi Kakuda

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
% Extract parameters from state vector
x = aircraft_state(1); 
y = aircraft_state(2); 
z = aircraft_state(3);
phi = aircraft_state4); 
theta = aircraft_state(5); 
psi = aircraft_state(6);
uE = aircraft_state(7); 
vE = aircraft_state(8); 
wE = aircraft_state(9);
p = aircraft_state(10); 
q = aircraft_state(11); 
r = aircraft_state(12);

% Classifying certain Parameters
omega = [p;q;r]; % Body Frame
Vb = [uE;vE;wE];
airspeed = norm(Vb,2);

% Rotation Matrix
R_EB = Rot_Mat(phi,theta,psi);    % Body to Earth
R_BE = R_EB.';                    % Earth to Body

% Motor --> Need to change
% f = motor_forces(:);
% Zc = -(f(1)+f(2)+f(3)+f(4));
% Lc = d/sqrt(2) * (-f(1) - f(2) + f(3) + f(4));
% Mc = d/sqrt(2) * ( f(1) - f(2) - f(3) + f(4));
% Nc = km * ( f(1) - f(2) + f(3) - f(4));

% Forces and Moments --> need to change
% F_thrust = [0; 0; Zc];                 % rotor resultant along body z
% F_g = m * (R_BE * [0;0;g]);       % gravity expressed in body
% F_tot = F_thrust + F_g + F_aero;
% M_tot = [Lc; Mc; Nc] + M_aero;

% Kinematics
position_dot = R_EB * Vb;
eul_dot = euler_kinematics(phi,theta,psi,omega);

% Dynamics
uvw_dot  = (F_tot - cross(omega, m * Vb)) / m; % Cross() is MatLab's cross function
pqr_dot  = I \ (M_tot - cross(omega, I*omega)); 

xdot = [position_dot; eul_dot; uvw_dot; pqr_dot];




