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
% Va = norm(Vb,2);

% Rotation Matrix
R_BE = Rot_Mat(phi,theta,psi);    % Body to Earth
R_EB = R_BE.';                    % Earth to Body

% Aerodynamic Forces and Moments
[Aero_F, Aero_M] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

% Calculate thrust and gravitational forces
F_thrust = [0; 0; aircraft_parameters.Zc]; % rotor resultant along body z
F_g = aircraft_parameters.m * (R_BE * [0; 0; aircraft_parameters.g]); % gravity expressed in body
F_tot = F_thrust + F_g + Aero_F; % Total forces
M_tot = [aircraft_parameters.Lc; aircraft_parameters.Mc; aircraft_parameters.Nc] + Aero_M; % Total moments

% Kinematics
position_dot = R_EB * Vb;
eul_dot = euler_kinematics(phi,theta,psi,omega);

% Dynamics
uvw_dot  = (F_tot - cross(omega, m * Vb)) / m; % Cross() is MatLab's cross function
pqr_dot  = I \ (M_tot - cross(omega, I*omega));

xdot = [position_dot; eul_dot; uvw_dot; pqr_dot];




