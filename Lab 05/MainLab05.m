%% LAB 05 | Lab Task
% Creators: Bridger, Natsumi, Drake, and Sayer

%% Notes
% This code uses NED positions for x,y,z in the inertial frame where z_e is 
% positive downward. With the Aircraft body frame using the standard convention
% of x_hat is out of the nose of the Aircraft, y_hat is out of the pilot's 
% right wing, and z_hat is direction positive downwards towards the Earth's core.

% Housekeeping code
clc; 
clear; 
close all;

%% Toggle Section
Problem2_1 = 0;
Problem2_2 = 0;
Problem2_3 = 1;
Problem3_1 = 0;
Problem3_2 = 0;

%% Overall Givens

% Call aircraft parameters
ttwistor();

% Figure Numbering and Coloring
fig21 = [1,2,3,4,5,6]; % Problem 2.1
fig22 = [7,8,9,10,11,12]; % Problem 2.2
fig23 = [13,14,15,16,17,18]; % Problem 2.3
fig31 = [19,20,21,22,23,24]; % Problem 3.1
fig32 = [25,26,27,28,29,30]; % Problem 3.2

col1 = 'b-'; % Problem 2.1
col2 = 'r-'; % Problem 2.2
col3 = 'k-'; % Problem 2.3
col4 = 'm-'; % Problem 3.1
col5 = 'g-'; % Problem 3.2

% Simulation Time
t_f = 100; % [s]

%% Problem 2.1
if Problem2_1 == 1
    % Trim state conditions
    x_e = 0;
    y_e = 0;
    z_e = -1609.34; % Z-direction is positive downwards
    phi = 0;
    theta = 0;
    psi = 0;
    u_e = 21;
    v_e = 0;
    w_e = 0;
    p = 0;
    q = 0;
    r = 0;
    
    AC_X21 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    AC_Surf = [0;0;0;0]; % zero deflection
    wind_inertial = [0;0;0]; % No wind
    
    % ODE45
    [t21, xdot_1] = ode45(@(t, x) AircraftEOM(t, x, AC_Surf, wind_inertial, aircraft_parameters), [0, t_f], AC_X21);

    ctrl_21 = repmat(AC_Surf.', length(t21), 1);  % Values of Control inputs with each row = [de, da, dr, dt]

    PlotAircraftSim(t21, xdot_1, ctrl_21, fig21, col1);
end

%% Problem 2.2 
if Problem2_2 == 1

    % Initial Conditions conditions
    x_e = 0;
    y_e = 0;
    z_e = -1800; % Z-direction is positive downwards
    phi = 0;
    theta = 0.02780; % [RADS]
    psi = 0;
    u_e = 20.99;
    v_e = 0;
    w_e = 0.5837;
    p = 0;
    q = 0;
    r = 0;
    
    AC_X22 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    AC_ctrl22 = [0.1079; 0; 0; 0.3182]; % u0 - Control input
    wind_inertial = [0;0;0]; % No wind

    odefun = @(t,x) AircraftEOM(t, x, AC_ctrl22, wind_inertial, aircraft_parameters);
    [t22, xdot_2] = ode45(odefun, [0, t_f], AC_X22);
    ctrl_22 = repmat(AC_ctrl22.', length(t22), 1);  % Values of Control inputs with each row = [de, da, dr, dt]

    PlotAircraftSim(t22, xdot_2, ctrl_22, fig22, col2);

end

%% Problem 2.3

if Problem2_3 == 1

    % Initial Conditions conditions
    x_e = 0;
    y_e = 0;
    z_e = -1800; % Z-direction is positive downwards
    phi = deg2rad(15);
    theta = deg2rad(-12); % [RADS]
    psi = deg2rad(270);
    u_e = 19;
    v_e = 3;
    w_e = -2;
    p = 0.08 * (pi/180); % Converts from deg/s to rad/s
    q = -0.2  * (pi/180); % Converts from deg/s to rad/s;
    r = 0;
    
    AC_X23 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    AC_ctrl23 = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3]; % Control imputs [de0, da0, dr0, dt0]
    wind_inertial = [0;0;0]; % No wind

    odefun = @(t,x) AircraftEOM(t, x, AC_ctrl23, wind_inertial, aircraft_parameters);
    [t23, xdot_3] = ode45(odefun, [0, t_f], AC_X23);
    
    ctrl_23 = repmat(AC_ctrl23.', length(t23), 1);  % Values of Control inputs with each row = [de, da, dr, dt]

    PlotAircraftSim(t23, xdot_3, ctrl_23, fig23, col3);

end

%% Problem 3.1

if Problem3_1 == 1

    % New simulation time
    t_f_sp = 3; % [s]

    % Initial Conditions conditions
    x_e = 0;
    y_e = 0;
    z_e = -1800; % Z-direction is positive downwards
    phi = 0;
    theta = 0.02780; % [RADS]
    psi = 0;
    u_e = 20.99;
    v_e = 0;
    w_e = 0.5837;
    p = 0;
    q = 0;
    r = 0;

    % Doublet set up
    doublet_size = deg2rad(15); % [rads]
    doublet_time = 0.25; % [s]
    
    AC_X31 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    AC_ctrl31 = [0.1079; 0; 0; 0.3182]; % Control input
    wind_inertial = [0;0;0]; % No wind

    % ode45
    [t31, xdot_4] = ode45(@(t, x) AircraftEOMDoublet(t, x, AC_ctrl31, doublet_size, doublet_time, wind_inertial, aircraft_parameters), [0, t_f_sp], AC_X31);

    % Control Values of Control inputs for Doublet Law (Controls)
    de_31 = zeros(size(t31)); % Elevator
    for i = 1:length(t31) 
        ti = t31(i);
        if ti > 0 && ti <= doublet_time
            de_31(i) = AC_ctrl31(1) + doublet_size;
        elseif ti > doublet_time && ti <= 2*doublet_time
            de_31(i) = AC_ctrl31(1) - doublet_size;
        else
            de_31(i) = AC_ctrl31(1);
        end
    end
    
    % Other control surfaces
    da_31 = AC_ctrl31(2) * ones(size(t31)); % Ailerons
    dr_31 = AC_ctrl31(3) * ones(size(t31)); % Rudder
    dt_31 = AC_ctrl31(4) * ones(size(t31)); % Throttle
    
    % Control Values Matrix
    ctrl_31 = [de_31, da_31, dr_31, dt_31]; % each row = [de, da, dr, dt]

    PlotAircraftSim(t31, xdot_4, ctrl_31, fig31, col4);

    % Natural Frequency and Damping Ratio for Short Period Mode
    %[w_n_31sp, zeta_31sp] = wn_zeta_funct_sp(z_e, u_e, aircraft_parameters);

end

%% Problem 3.2

if Problem3_2 == 1

    % New simulation time
    t_f_lp = 100; % [s]

    % Initial Conditions conditions
    x_e = 0;
    y_e = 0;
    z_e = -1800; % Z-direction is positive downwards
    phi = 0;
    theta = 0.02780; % [RADS]
    psi = 0;
    u_e = 20.99;
    v_e = 0;
    w_e = 0.5837;
    p = 0;
    q = 0;
    r = 0;

    % Doublet set up
    doublet_size = deg2rad(15); % [rads]
    doublet_time = 0.25; % [s]
    
    AC_X32 = [x_e; y_e; z_e; phi; theta; psi; u_e; v_e; w_e; p; q; r];
    AC_ctrl = [0.1079; 0; 0; 0.3182]; % u0 - Control input
    wind_inertial = [0;0;0]; % No wind

    odefun = @(t,AC_X32) AircraftEOMDoublet(t_f_lp, AC_X32, AC_ctrl, doublet_size, doublet_time, wind_inertial, aircraft_parameters);
    [t32, xdot_5] = ode45(odefun, [0, t_f_lp], AC_X32);

    % Control Values of Control inputs for Doublet Law (Controls)
    de_32 = zeros(size(t32)); % Elevator
    for i = 1:length(t32) 
        ti = t32(i);
        if ti > 0 && ti <= doublet_time
            de_32(i) = AC_ctrl(1) + doublet_size;
        elseif ti > doublet_time && ti <= 2*doublet_time
            de_32(i) = AC_ctrl(1) - doublet_size;
        else
            de_32(i) = AC_ctrl(1);
        end
    end
    
    % Other control surfaces
    da_32 = AC_ctrl(2) * ones(size(t32)); % Ailerons
    dr_32 = AC_ctrl(3) * ones(size(t32)); % Rudder
    dt_32 = AC_ctrl(4) * ones(size(t32)); % Throttle
    
    % Control Values Matrix
    ctrl_32 = [de_32, da_32, dr_32, dt_32]; % each row = [de, da, dr, dt]

    PlotAircraftSim(t32, xdot_5, ctrl_32, fig32, col5);

    % Natural Frequency and Damping Ratio for Phugiod Mode
    %[w_n_32ph, zeta_32ph] = wn_zeta_funct_ph(z_e, u_e, aircraft_parameters);

end


