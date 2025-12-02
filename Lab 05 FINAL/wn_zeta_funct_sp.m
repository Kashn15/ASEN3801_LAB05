%% Natural Frequency and Damping Ratio Estimation for Short Period Mode
% Creator: Bridger Cushman

function [w_n, zeta] = wn_zeta_funct_sp(z_e, u0, aircraft_parameters)

    [~,~,~,rho] = atmosisa(-z_e); % Density [kg/m^3]
    M_w = .5*rho*u0*aircraft_parameters.S*aircraft_parameters.c*aircraft_parameters.Cmalpha;
    M_q = .25*rho*u0*(aircraft_parameters.c^2)*aircraft_parameters.S*aircraft_parameters.Cmq;
    Czalpha = -aircraft_parameters.CD0-aircraft_parameters.CLalpha;
    Z_w = .5*rho*u0*aircraft_parameters.S*Czalpha;
    M_wdot = .25*rho*(aircraft_parameters.c^2)*aircraft_parameters.S*aircraft_parameters.Cmalphadot;

    w_n = sqrt(-(1/aircraft_parameters.Iy)*(u0*M_w - (M_q*Z_w/aircraft_parameters.m)));
    zeta = -(1/(2*w_n))*((Z_w/aircraft_parameters.m) +(1/aircraft_parameters.Iy)*(M_q+M_wdot*u0));

end