%% Natural Frequency and Damping Ratio Estimation for Phugoid Mode
% Creator: Bridger Cushman

function [w_n, zeta] = wn_zeta_funct(z_e, u0, aircraft_parameters)

    [~,~,~,rho] = atmosisa(-z_e); % Density [kg/m^3]
    Z_u = -rho*u0*aircraft_parameters.S*aircraft_parameters.CL0; %probably should be using u0
    w_n = sqrt(-(Z_u*aircraft_parameters.g)/(aircraft_parameters.m*u0));
    zeta = (1/sqrt(2))*(aircraft_parameters.CD0/aircraft_parameters.CL0);

end
