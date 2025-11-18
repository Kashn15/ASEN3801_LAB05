%% LAB 05 | Lab Task
% Creator: Natsumi Kakuda

% Housekeeping
clc; clear; close all;

%% Toggle Section
ttwistor();

[~,~,~,density_SI] = atmosisa(1609.34);
aircraft_parameters.density = density_SI;
x0=0;
y0=0;
z0=-1609.34;
phi0=0;
theta0=0;
psi0=0;
u0=21;
v0=0;
w0=0;
p0=0;
q0=0;
r0=0;
initial_aircraft_state = [x0;y0;z0;phi0;theta0;psi0;u0;v0;w0;p0;q0;r0];

de0=0;
da0=0;
dr0=0;
dt0=0;

aircraft_surfaces = [de0;da0;dr0;dt0];

wind_inertial = [0;0;0];


%% 
eom_function_handle = @(t,aircraft_state) AircraftEOM(t,aircraft_state,aircraft_surfaces,wind_inertial,aircraft_parameters);

tspan = [0 10];
ops = [];
[time1, state1] = ode45(eom_function_handle,tspan,initial_aircraft_state,ops);
control_input_array = aircraft_surfaces * (ones(size(time1)))';
control_input_array = control_input_array';
%control_input_array = [];
fig = [1,2,3,4,5,6];
col = 'b-';
PlotAircraftSim(time1,state1,control_input_array,fig,col);
