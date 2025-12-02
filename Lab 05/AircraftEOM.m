%% AircraftEOM Function
% Creators: Bridger, Natsumi, Drake, and Sayer

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

% Notes
% Earth frame: NED (z positive Down)
% Body frame : x forward (out of nose), y right (out of pilot's right wing), z down

% Extract parameters from state vector
xE = aircraft_state(1); 
yE = aircraft_state(2); 
zE = aircraft_state(3);
phi = aircraft_state(4); 
theta = aircraft_state(5); 
psi = aircraft_state(6);
u = aircraft_state(7); 
v = aircraft_state(8); 
w = aircraft_state(9);
p = aircraft_state(10); 
q = aircraft_state(11); 
r = aircraft_state(12);

% Inertia
Ix = aircraft_parameters.Ix;
Ixz = aircraft_parameters.Ixz;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;

% Inertia Matrix Set up
% I = [Ix, 0, Ixz; 0, Iy, 0; Ixz, 0, Iz];

% Gamma 
Gamma = Ix*Iz - Ixz^2;
Gamma1 = (Ixz*(Ix-Iy)+Ixz^2)/Gamma;
Gamma2 = (Iz*(Iz-Iy)+Ixz^2)/Gamma;
Gamma3 = Iz / Gamma;
Gamma4 = Ixz/Gamma;
Gamma5 = (Ix - Iz) / Gamma;
Gamma6 = Ixz / Iy;
Gamma7 = (Ix*(Ix-Iy)+Ixz^2)/Gamma;
Gamma8 = Ix/Gamma;

% Omega and Vb Set up
omega = [p;q;r]; % Body Frame
Vb = [u;v;w];

% Rotation Matrices
R_BE = Rot_Mat(phi,theta,psi);    % Body to Earth

% Density Calculation
h = abs(zE); % We want magnitude since our frame defines zE as positive downwards
[~,~,~,rho] = atmosisa(h); % Density [kg/m^3]
density = rho;

% Aerodynamic Forces and Moments
[Aero_F, Aero_M] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
X = Aero_F(1);
Y = Aero_F(2);
Z = Aero_F(3);
L = Aero_M(1);
M = Aero_M(2);
N = Aero_M(3);

% Calculate thrust and gravitational forces
% F_g = aircraft_parameters.m * (R_EB * [0; 0; aircraft_parameters.g]);
% F_tot = Aero_F + F_g; % Total forces
% M_tot = Aero_M; % Total moments

% Kinematics
pos_dot = R_BE * Vb;
eul_dot = euler_kinematics(phi,theta,psi,omega);

% Extracting Velocity
u_E = pos_dot(1);
v_E = pos_dot(2);
w_E = pos_dot(3);

% Dynamics
p_dot = (Gamma1*p*q - Gamma2*q*r) + (Gamma3*L+Gamma4*N);
q_dot = (Gamma5*p*r - Gamma6*((p^2)-(r^2)))+(1/Iy)*M;
r_dot = (Gamma7*p*q - Gamma1*q*r) + (Gamma4*L+Gamma8*N);
pqr_dot = [p_dot; q_dot; r_dot];

u_dot = (r*v_E- q*w_E)+ aircraft_parameters.g *(-sin(theta))+ (X/aircraft_parameters.m);
v_dot = (p*w_E + r*u_E) + aircraft_parameters.g * cos(theta) * sin(phi) + (Y/aircraft_parameters.m);
w_dot = (q*u_E - p*v_E) + aircraft_parameters.g * cos(theta) * cos(phi) + (Z/aircraft_parameters.m);
uvw_dot = [u_dot; v_dot; w_dot];

xdot = [pos_dot; eul_dot; uvw_dot; pqr_dot];






