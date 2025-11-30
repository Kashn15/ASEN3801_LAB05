%% AircraftEOM Function
% Creator: Natsumi Kakuda

function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters)

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

% Trim Controls Extraction
de_trim = aircraft_surfaces(1);
da = aircraft_surfaces(2);
dr = aircraft_surfaces(3);
dt = aircraft_surfaces(4);

% Elevator Doublet Logic
if time > 0 && time <= doublet_time
    de = de_trim + doublet_size;
elseif time > doublet_time && time <= 2*doublet_time
    de = de_trim - doublet_size;
elseif time > 2*doublet_time
    de = de_trim;
end

% Aircraft Control Vector
aircraft_surfaces = [de; da; dr; dt];

% Recall Exisiting Aircraft EOM
xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters);

end



